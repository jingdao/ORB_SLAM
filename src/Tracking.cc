/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Tracking.h"
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>

#include"ORBmatcher.h"
#include"FramePublisher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<fstream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>

ros::Subscriber subLidar;
geometry_msgs::PoseStamped initialHectorPose_msg;
geometry_msgs::PoseStamped hectorPose_msg;
sensor_msgs::PointCloud hectorCloud_msg;
cv::Mat hectorPose = cv::Mat::eye(4,4,CV_32F);
double imu_time = 0;
Eigen::Quaternionf imu_pose;
double hector_time_cur=0,hector_time_ini=0;
bool wait_hector = false;
bool ORB_SLAM::Tracking::use_homography = false;
bool ORB_SLAM::Tracking::hack_pose = true;
bool ORB_SLAM::Tracking::debug_tracking = false;
bool ORB_SLAM::Tracking::debug_optimizer = false;
bool ORB_SLAM::Tracking::optim_fix_map = false;
bool ORB_SLAM::Tracking::optim_fix_pose = true;
bool ORB_SLAM::Tracking::optim_adjust_scale = false;
bool ORB_SLAM::Tracking::save_initial_map = false;
bool ORB_SLAM::Tracking::minimal_build = false;
bool ORB_SLAM::Tracking::use_imu = false;
double ORB_SLAM::Tracking::minOffset = 0.1;
double ORB_SLAM::Tracking::tracking_threshold = 10;
double ORB_SLAM::Tracking::tracking_threshold_local = 30;
Eigen::Vector3d ORB_SLAM::Tracking::tcl(0,0,0);
cv::Mat Rcw; // Current Camera Rotation
cv::Mat tcw; // Current Camera Translation
using namespace std;

namespace ORB_SLAM
{

Eigen::Matrix<double,3,3> poseToRotation(geometry_msgs::PoseStamped msg) {
	double qx = msg.pose.orientation.x;
	double qy = msg.pose.orientation.y;
	double qz = msg.pose.orientation.z;
	double qw = msg.pose.orientation.w;
	Eigen::Matrix<double,3,3> R;
	R << 1 - 2*qy*qy - 2 * qz*qz,
	2*qx*qy - 2 * qz*qw,
	2*qx*qz + 2 * qy*qw,
	2*qx*qy + 2 * qz*qw,
	1 - 2*qx*qx - 2 * qz*qz,
	2*qy*qz - 2 * qx*qw,
	2*qx*qz - 2 * qy*qw,
	2*qy*qz + 2 * qx*qw,
	1 - 2*qx*qx - 2 * qy*qy;
	return R;
}

Eigen::Vector3d poseToTranslation(geometry_msgs::PoseStamped msg) {
	Eigen::Vector3d T;
	T << 
		msg.pose.position.x,
		msg.pose.position.y,
		msg.pose.position.z;
	return T;
}

double poseToOffset(geometry_msgs::PoseStamped p1,geometry_msgs::PoseStamped p2) {
	double d = 0;
	d += (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x);
	d += (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y);
	d += (p1.pose.position.z - p2.pose.position.z) * (p1.pose.position.z - p2.pose.position.z);
	return sqrt(d);
}

Eigen::Matrix<double,4,4> poseToTransformation(geometry_msgs::PoseStamped msg) {
	Eigen::Matrix<double,3,3> Rcl;
//	Eigen::Vector3d tcl;
	Rcl << 1,0,0,0,0,-1,0,1,0;
//	tcl << 0,-0.25,-0.18;
	Eigen::Matrix<double,3,3> Rwl = poseToRotation(msg);
	Eigen::Vector3d twl = poseToTranslation(msg);
	Eigen::Matrix<double,3,3> Rcw = Rcl * Rwl.transpose();
	Eigen::Vector3d tcw = - Rcl * Rwl.transpose() * twl + Tracking::tcl;
	Eigen::Matrix<double,4,4> Tcw = Eigen::Matrix<double,4,4>::Identity();
	Tcw.block(0,0,3,3) = Rcw;
	Tcw.block(0,3,3,1) = tcw;
	return Tcw;
}

static float invSqrt(float x) {
	float xhalf = 0.5f * x;
	union {
		float x;
		int i;
	} u;
	u.x = x;
	u.i = 0x5f3759df - (u.i >> 1);
	/* The next line can be repeated any number of times to increase accuracy */
	u.x = u.x * (1.5f - xhalf * u.x * u.x);
	return u.x;
}

Eigen::Quaternionf madgwickAHRSupdateIMU(Eigen::Quaternionf q, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		float gain = 0.1;
		qDot1 -= gain * s0;
		qDot2 -= gain * s1;
		qDot3 -= gain * s2;
		qDot4 -= gain * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	return Eigen::Quaternionf(q0,q1,q2,q3);
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg) {
	hectorPose_msg = *poseMsg;
	Eigen::Matrix<double,4,4> Tcw = poseToTransformation(hectorPose_msg);
	hectorPose = Converter::toCvMat(Tcw);
	hector_time_cur = poseMsg->header.stamp.toSec();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& imuMsg) {
	double t = imuMsg->header.stamp.toSec();
	double gx = imuMsg->angular_velocity.x;
	double gy = imuMsg->angular_velocity.y;
	double gz = imuMsg->angular_velocity.z;
	double ax = imuMsg->linear_acceleration.x;
	double ay = imuMsg->linear_acceleration.y;
	double az = imuMsg->linear_acceleration.z;
	if (imu_time > 0) {
		imu_pose = madgwickAHRSupdateIMU(imu_pose,gx,gy,gz,ax,ay,az,t-imu_time);
	} else {
		double sign = az <0 ? -1 : 1;
		double roll = atan2(ay, sign * sqrt(ax*ax + az*az));
		double pitch = -atan2(ax, sqrt(ay*ay + az*az));
		float q0 = cos(roll/2) * cos(pitch/2);
		float q1 = sin(roll/2) * cos(pitch/2);
		float q2 = cos(roll/2) * sin(pitch/2);
		imu_pose = Eigen::Quaternionf(q0,q1,q2,0);
	}
	imu_time = t;
}

void cloud_callback(const sensor_msgs::PointCloudConstPtr& lidarMsg) {
	hectorCloud_msg = *lidarMsg;
	subLidar.shutdown();
}

Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher *pFramePublisher, MapPublisher *pMapPublisher, Map *pMap, string strSettingPath):
    mState(NO_IMAGES_YET), mpORBVocabulary(pVoc), mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher), mpMap(pMap),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mbMotionModel(false)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = 18*fps/30;


    cout << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fastTh = fSettings["ORBextractor.fastTh"];    
    int Score = fSettings["ORBextractor.nScoreType"];

    assert(Score==1 || Score==0);

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    mpIniORBextractor = new ORBextractor(nFeatures*2,1.2,8,Score,fastTh);  

    int nMotion = fSettings["UseMotionModel"];
    mbMotionModel = nMotion;

    if(mbMotionModel)
    {
        mVelocity = cv::Mat::eye(4,4,CV_32F);
        cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
        cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;


    tf::Transform tfT;
    tfT.setIdentity();
    mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

void Tracking::Run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &Tracking::GrabImage, this);
	ros::Subscriber subPose = nodeHandler.subscribe("/slam_out_pose",1,pose_callback);
	if (use_imu)
		ros::Subscriber subImu = nodeHandler.subscribe("/asctec_proc/imu",1,imu_callback);
	subLidar = nodeHandler.subscribe("/slam_cloud",1,cloud_callback);

    ros::spin();
}

void Tracking::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	//discard image if not synced with hector update
	if (msg->header.stamp.toSec() > hector_time_cur + 0.1) {
		if (debug_tracking)
			printf("discard %f %f\n",msg->header.stamp.toSec(),hector_time_cur);
		usleep(100000);
		return;
	}
    cv::Mat im;

    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);

    if(cv_ptr->image.channels()==3)
    {
        if(mbRGB)
            cvtColor(cv_ptr->image, im, CV_RGB2GRAY);
        else
            cvtColor(cv_ptr->image, im, CV_BGR2GRAY);
    }
    else if(cv_ptr->image.channels()==1)
    {
        cv_ptr->image.copyTo(im);
    }

    if(mState==WORKING || mState==LOST)
        mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpORBextractor,mpORBVocabulary,mK,mDistCoef);
    else
        mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpIniORBextractor,mpORBVocabulary,mK,mDistCoef);
    // Depending on the state of the Tracker we perform different tasks

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
		Initialize();
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        if(mState==WORKING && !RelocalisationRequested())
        {
            if(!mbMotionModel || mpMap->KeyFramesInMap()<4 || mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                bOK = TrackPreviousFrame();
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackPreviousFrame();
            }
        }
        else
        {
            bOK = Relocalisation();
        }

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK) {
            bOK = TrackLocalMap();
		}

        // If tracking were good, check if we insert a keyframe
        if(bOK) {
            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);
			//write to file
			if (save_initial_map) {
				FILE* orb_pose = fopen("/home/jd/Documents/vslam/orb/orb_pose.txt","a");
				Eigen::Matrix<double,3,3> R = Converter::toMatrix3d(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
				Eigen::Quaterniond Q(R);
				fprintf(orb_pose,"%f %f %f %f %f %f %f %f\n",mCurrentFrame.mTimeStamp,
					mCurrentFrame.mTcw.at<float>(0,3),mCurrentFrame.mTcw.at<float>(1,3),mCurrentFrame.mTcw.at<float>(2,3),
					Q.w(),Q.x(),Q.y(),Q.z());
				fclose(orb_pose);
			}

            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(size_t i=0; i<mCurrentFrame.mvbOutlier.size();i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }

        if(bOK)
            mState = WORKING;
        else
            mState=LOST;

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }

        // Update motion model
        if(mbMotionModel)
        {
            if(bOK && !mLastFrame.mTcw.empty())
            {
                cv::Mat LastRwc = mLastFrame.mTcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat Lasttwc = -LastRwc*mLastFrame.mTcw.rowRange(0,3).col(3);
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                LastRwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                Lasttwc.copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();
        }

        mLastFrame = Frame(mCurrentFrame);
     }       

    // Update drawer
    mpFramePublisher->Update(this);

    if(!Tracking::minimal_build && !mCurrentFrame.mTcw.empty()) {
        cv::Mat Rwc = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*mCurrentFrame.mTcw.rowRange(0,3).col(3);
        tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                        Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                        Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
        tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

        tf::Transform tfTcw(M,V);

        mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
    }

}


void Tracking::FirstInitialization()
{
    //We ensure a minimum ORB features to continue, otherwise discard frame
    if(hector_time_cur > 0 && mCurrentFrame.mvKeys.size()>100)
    {
		hector_time_ini = hector_time_cur;
		initialHectorPose_msg = hectorPose_msg;
        mInitialFrame = Frame(mCurrentFrame);
        mLastFrame = Frame(mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
        for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
            mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

        if(mpInitializer)
            delete mpInitializer;

        mpInitializer =  new Initializer(mCurrentFrame,1.0,200);


        mState = INITIALIZING;
    }
}

void Tracking::Initialize()
{
	if (poseToOffset(initialHectorPose_msg,hectorPose_msg) < Tracking::minOffset) {
		return;
	}
    // Check if current frame has enough keypoints, otherwise reset initialization process
    if(mCurrentFrame.mvKeys.size()<=100)
    {
        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
        mState = NOT_INITIALIZED;
        return;
    }   

    // Find correspondences
    ORBmatcher matcher(0.9,true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

    // Check if there are enough correspondences
    if(nmatches<100)
    {
        mState = NOT_INITIALIZED;
        return;
    }  

//    cv::Mat Rcw; // Current Camera Rotation
//    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

	if (Tracking::use_homography) {
		if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
		{
			for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
			{
				if(mvIniMatches[i]>=0 && !vbTriangulated[i])
				{
					mvIniMatches[i]=-1;
					nmatches--;
				}           
			}
			CreateInitialMap(Rcw,tcw);
		}
	} else {
		CreateInitialMap(Rcw,tcw);
	}

}

void Tracking::CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw)
{
    // Set Frame Poses
    mInitialFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    mCurrentFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
	if (!Tracking::hack_pose) {
		Rcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
		tcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).col(3));
	}

    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

	//linearly interpolate pose
	double x1 = initialHectorPose_msg.pose.position.x;
	double y1 = initialHectorPose_msg.pose.position.y;
	double z1 = initialHectorPose_msg.pose.position.z;
	double x2 = hectorPose_msg.pose.position.x;
	double y2 = hectorPose_msg.pose.position.y;
	double z2 = hectorPose_msg.pose.position.z;
	double t1 = initialHectorPose_msg.header.stamp.toSec(); 
	double t2 = hectorPose_msg.header.stamp.toSec(); 
	double ti = mInitialFrame.mTimeStamp;
	double tf = mCurrentFrame.mTimeStamp;
	initialHectorPose_msg.pose.position.x = (x1*(t2-ti) + x2*(ti-t1)) / (t2-t1);
	initialHectorPose_msg.pose.position.y = (y1*(t2-ti) + y2*(ti-t1)) / (t2-t1);
	initialHectorPose_msg.pose.position.z = (z1*(t2-ti) + z2*(ti-t1)) / (t2-t1);
	hectorPose_msg.pose.position.x = (x1*(t2-tf) + x2*(tf-t1)) / (t2-t1);
	hectorPose_msg.pose.position.y = (y1*(t2-tf) + y2*(tf-t1)) / (t2-t1);
	hectorPose_msg.pose.position.z = (z1*(t2-tf) + z2*(tf-t1)) / (t2-t1);

	//Write to pose file
	FILE* pose_stamped,*key_match,*cvkey_match,*map_point,*orb_pose,*lidar_map;
	if (save_initial_map) {
		pose_stamped = fopen("/home/jd/Documents/vslam/orb/pose_stamped.txt","w");
		key_match = fopen("/home/jd/Documents/vslam/orb/key.match","w");
		cvkey_match = fopen("/home/jd/Documents/vslam/orb/cvkey.match","w");
		map_point = fopen("/home/jd/Documents/vslam/orb/map_point.txt","w");
		orb_pose = fopen("/home/jd/Documents/vslam/orb/orb_pose.txt","w");
		lidar_map = fopen("/home/jd/Documents/vslam/orb/lidar_map.txt","w");
		fclose(orb_pose);
		fprintf(pose_stamped,"%f %f %f %f %f %f %f %f\n",
			initialHectorPose_msg.header.stamp.toSec(),
			initialHectorPose_msg.pose.position.x,
			initialHectorPose_msg.pose.position.y,
			initialHectorPose_msg.pose.position.z,
			initialHectorPose_msg.pose.orientation.w,
			initialHectorPose_msg.pose.orientation.x,
			initialHectorPose_msg.pose.orientation.y,
			initialHectorPose_msg.pose.orientation.z);
		fprintf(pose_stamped,"%f %f %f %f %f %f %f %f\n",
			hectorPose_msg.header.stamp.toSec(),
			hectorPose_msg.pose.position.x,
			hectorPose_msg.pose.position.y,
			hectorPose_msg.pose.position.z,
			hectorPose_msg.pose.orientation.w,
			hectorPose_msg.pose.orientation.x,
			hectorPose_msg.pose.orientation.y,
			hectorPose_msg.pose.orientation.z);
		for (unsigned int i=0; i<hectorCloud_msg.points.size(); i++) {
			fprintf(lidar_map,"%f %f %f\n",hectorCloud_msg.points[i].x,hectorCloud_msg.points[i].y,0.0);
		}
		fclose(lidar_map);
		fclose(pose_stamped);
	}

	if (Tracking::hack_pose) {
		cout << "time_ini " << mInitialFrame.mTimeStamp << "\n"; 
		cout << "time_cur " << mCurrentFrame.mTimeStamp << "\n"; 
		cout << "time_ini " << hector_time_ini << "\n"; 
		cout << "time_cur " << hector_time_cur << "\n"; 
		cout << "pKFcur " << pKFcur->GetPose() << "\n";

		Eigen::Matrix<double,4,4> Tcw1 = poseToTransformation(initialHectorPose_msg);
		Eigen::Matrix<double,4,4> Tcw2 = poseToTransformation(hectorPose_msg);
		vector<Eigen::Vector3d> scan;
		if (Tracking::optim_adjust_scale) {
			for (size_t i=0;i<hectorCloud_msg.points.size();i++)
				scan.push_back(Eigen::Vector3d(hectorCloud_msg.points[i].x,hectorCloud_msg.points[i].y,hectorCloud_msg.points[i].z));
		}
		Optimizer::Triangulation(&mInitialFrame,&mCurrentFrame,&mvIniMatches,Tcw1,Tcw2,&mvIniP3D,&scan);
		pKFini->SetPose(Converter::toCvMat(Tcw1));
		pKFcur->SetPose(Converter::toCvMat(Tcw2));
		cout << "pKFini " << pKFini->GetPose() << "\n";
		cout << "pKFcur " << pKFcur->GetPose() << "\n";

		// Create MapPoints and asscoiate to keyframes
		for(size_t i=0; i<mvIniMatches.size();i++) {
			if(mvIniMatches[i]<0)
				continue;
			//write to match file
			if (save_initial_map) {
				cv::Point3f worldPos = mvIniP3D[i];
				fprintf(map_point,"%f %f %f\n",worldPos.x,worldPos.y,worldPos.z);
				fprintf(key_match,"0 %f %f 1 %f %f\n",
					mInitialFrame.mvKeysUn[i].pt.x,mInitialFrame.mvKeysUn[i].pt.y,
					mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x,mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y);
				fprintf(cvkey_match,"%f %f %d %f %f %d\n",
					mInitialFrame.mvKeysUn[i].pt.x,mInitialFrame.mvKeysUn[i].pt.y,mInitialFrame.mvKeysUn[i].octave,
					mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x,mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y,mCurrentFrame.mvKeysUn[mvIniMatches[i]].octave);
			}
			//Create MapPoint.
			cv::Mat worldPos(mvIniP3D[i]);
			MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);
			pKFini->AddMapPoint(pMP,i);
			pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
			pMP->AddObservation(pKFini,i);
			pMP->AddObservation(pKFcur,mvIniMatches[i]);
			pMP->ComputeDistinctiveDescriptors();
			pMP->UpdateNormalAndDepth();
			//Fill Current Frame structure
			mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
			//Add to Map
			mpMap->AddMapPoint(pMP);
		}
		if (save_initial_map) {
			cv::Mat im1 = mInitialFrame.im.clone();
			cv::Mat im2 = mCurrentFrame.im.clone();
			cv::cvtColor(im1,im1,CV_GRAY2BGR);
			cv::cvtColor(im2,im2,CV_GRAY2BGR);
			const float r = 5;
			cv::Point2f pt1,pt2;
			for (size_t i=0;i<mvIniMatches.size();i++) {
				if (mvIniMatches[i] >= 0) {
					pt1.x=mInitialFrame.mvKeysUn[i].pt.x-r;
					pt1.y=mInitialFrame.mvKeysUn[i].pt.y-r;
					pt2.x=mInitialFrame.mvKeysUn[i].pt.x+r;
					pt2.y=mInitialFrame.mvKeysUn[i].pt.y+r;
					cv::rectangle(im1,pt1,pt2,cv::Scalar(0,255,0));
					cv::circle(im1,mInitialFrame.mvKeysUn[i].pt,2,cv::Scalar(0,255,0),-1);
					pt1.x=mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x-r;
					pt1.y=mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y-r;
					pt2.x=mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x+r;
					pt2.y=mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y+r;
					cv::rectangle(im2,pt1,pt2,cv::Scalar(0,255,0));
					cv::circle(im2,mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt,2,cv::Scalar(0,255,0),-1);
				}
			}
			cv::imwrite("/home/jd/Documents/vslam/orb/initial_frame.jpg",im1);
			cv::imwrite("/home/jd/Documents/vslam/orb/current_frame.jpg",im2);
		}
		// Update Connections
		pKFini->UpdateConnections();
		pKFcur->UpdateConnections();
		// Bundle Adjustment
		ROS_INFO("New Map created with %d points (%lu matches)",mpMap->MapPointsInMap(),mvIniMatches.size());
//		Optimizer::GlobalBundleAdjustemnt(mpMap,20);

	} else {
		// Create MapPoints and asscoiate to keyframes
		for(size_t i=0; i<mvIniMatches.size();i++) {
			if(mvIniMatches[i]<0)
				continue;
			//write to match file
			if (save_initial_map) {
				fprintf(key_match,"0 %f %f 1 %f %f\n",
					mInitialFrame.mvKeysUn[i].pt.x,mInitialFrame.mvKeysUn[i].pt.y,
					mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x,mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y);
			}
			//Create MapPoint.
			cv::Mat worldPos(mvIniP3D[i]);
			MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);
			pKFini->AddMapPoint(pMP,i);
			pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
			pMP->AddObservation(pKFini,i);
			pMP->AddObservation(pKFcur,mvIniMatches[i]);
			pMP->ComputeDistinctiveDescriptors();
			pMP->UpdateNormalAndDepth();
			//Fill Current Frame structure
			mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
			//Add to Map
			mpMap->AddMapPoint(pMP);
		}
		// Update Connections
		pKFini->UpdateConnections();
		pKFcur->UpdateConnections();
		// Bundle Adjustment
		ROS_INFO("New Map created with %d points (%lu matches)",mpMap->MapPointsInMap(),mvIniMatches.size());
		Optimizer::GlobalBundleAdjustemnt(mpMap,20);

		// Set median depth to 1
		float medianDepth = pKFini->ComputeSceneMedianDepth(2);
		float invMedianDepth = 1.0f/medianDepth;

		if(medianDepth<0 || pKFcur->TrackedMapPoints()<100)
		{
			ROS_INFO("Wrong initialization, reseting...");
			Reset();
			return;
		}

		// Scale initial baseline
		cv::Mat Tc2w = pKFcur->GetPose();
		Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
		pKFcur->SetPose(Tc2w);

		// Scale points
		vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
		for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
		{
			if(vpAllMapPoints[iMP])
			{
				MapPoint* pMP = vpAllMapPoints[iMP];
				pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
				cv::Mat worldPos = pMP->GetWorldPos();
				if (save_initial_map) {
					fprintf(map_point,"%f %f %f\n",worldPos.at<float>(0,0),worldPos.at<float>(1,0),worldPos.at<float>(2,0));
				}
			}
		}
	}
	if (save_initial_map) {
		fclose(key_match);
		fclose(cvkey_match);
		fclose(map_point);
	}

	mpLocalMapper->InsertKeyFrame(pKFini);
	mpLocalMapper->InsertKeyFrame(pKFcur);

    mInitialFrame.mTcw = pKFini->GetPose().clone();
    mCurrentFrame.mTcw = pKFcur->GetPose().clone();
    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());

    mState=WORKING;
}


bool Tracking::TrackPreviousFrame()
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;

    int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);
	int initialMatches = nmatches;

    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
        }
    }

    mLastFrame.mTcw.copyTo(mCurrentFrame.mTcw);
    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        // Optimize pose with correspondences
		if (optim_fix_map)
        	Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
		else
        	Optimizer::MapOptimization(&mCurrentFrame,hectorPose);

        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
    }
    else {
		//Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);
	}


    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    if(nmatches<Tracking::tracking_threshold)
        return false;

    // Optimize pose again with all correspondences
	if (optim_fix_map)
    	Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
	else
		Optimizer::MapOptimization(&mCurrentFrame,hectorPose);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }

	if (debug_tracking)
		printf("track previous frame: %d %d\n",initialMatches,nmatches);
    return nmatches>=Tracking::tracking_threshold;
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Compute current pose by motion model
    mCurrentFrame.mTcw = mVelocity*mLastFrame.mTcw;

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,15);
	int initialMatches = nmatches;

    if(nmatches<20)
       return false;

    // Optimize pose with all correspondences
	if (debug_tracking)
		printf("timestamps: %f %f\n",mCurrentFrame.mTimeStamp,hector_time_cur);
	if (optim_fix_map)
    	Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
	else
    	Optimizer::MapOptimization(&mCurrentFrame,hectorPose);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }
	if (debug_tracking)
		printf("track with motion model: %d %d\n",initialMatches,nmatches);
    return nmatches>=10;
}

bool Tracking::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    UpdateReference();

    // Search Local MapPoints
    SearchReferencePointsInFrustum();

    // Optimize Pose
	if (optim_fix_map)
    	mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
	else
    	mnMatchesInliers = Optimizer::MapOptimization(&mCurrentFrame,hectorPose);
	if (debug_tracking)
		printf("track local map: %d\n",mnMatchesInliers);

    // Update MapPoints Statistics
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<Tracking::tracking_threshold_local+20)
        return false;

    if(mnMatchesInliers<Tracking::tracking_threshold_local)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
    // Condition 2: Less than 90% of points than reference keyframe and enough inliers
    const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpLocalMapper->InsertKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchReferencePointsInFrustum()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    mCurrentFrame.UpdatePoseMatrices();

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;        
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }    


    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateReference()
{    
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();
}

void Tracking::UpdateReferencePoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(size_t i=0, iend=mCurrentFrame.mvpMapPoints.size(); i<iend;i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    int max=0;
    KeyFrame* pKFmax=NULL;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

    }

    mpReferenceKF = pKFmax;
}

bool Tracking::Relocalisation()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;
    if(!RelocalisationRequested())
        vpCandidateKFs= mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }        
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood;
				if (optim_fix_map)
					nGood = Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
				else
					nGood = Optimizer::MapOptimization(&mCurrentFrame,hectorPose);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mCurrentFrame.mvbOutlier.size(); io<ioend; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
						if (optim_fix_map)
							nGood = Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
						else
							nGood = Optimizer::MapOptimization(&mCurrentFrame,hectorPose);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mCurrentFrame.mvpMapPoints.size(); ip<ipend; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
								if (optim_fix_map)
									nGood = Optimizer::PoseOptimization(&mCurrentFrame,hectorPose);
								else
									nGood = Optimizer::MapOptimization(&mCurrentFrame,hectorPose);

                                for(size_t io =0; io<mCurrentFrame.mvbOutlier.size(); io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {                    
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::ForceRelocalisation()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool Tracking::RelocalisationRequested()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}


void Tracking::Reset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = false;
        mbReseting = true;
    }

    // Wait until publishers are stopped
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(mbPublisherStopped)
                break;
        }
        r.sleep();
    }

    // Reset Local Mapping
	mpLocalMapper->RequestReset();
    // Reset Loop Closing
	if (mpLoopClosing)
    	mpLoopClosing->RequestReset();
    // Clear BoW Database
    mpKeyFrameDB->clear();
    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NOT_INITIALIZED;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbReseting = false;
    }
}

void Tracking::CheckResetByPublishers()
{
    bool bReseting = false;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        bReseting = mbReseting;
    }

    if(bReseting)
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = true;
    }

    // Hold until reset is finished
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(!mbReseting)
            {
                mbPublisherStopped=false;
                break;
            }
        }
        r.sleep();
    }
}

} //namespace ORB_SLAM
