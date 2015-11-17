#!/bin/bash

for f in `find -name CMakeCache.txt`
do
	cat $f | sed s/":\/"/"\/"/g > $f-2
	mv $f-2 $f
done
