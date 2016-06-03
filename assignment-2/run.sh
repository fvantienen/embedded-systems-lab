#!/bin/bash
for k in klomp tiger square
do
	for i in `seq 0 100`
	do
		~/powercycle.sh
		echo "Running magnitude $i $k"
		./canny_edge canny_edge.out pics/$k.pgm 100 100 $i >> ./mag_$k.csv
	done

	for i in `seq 0 100`
	do
		~/powercycle.sh
		echo "Running derivative $i $k"
		./canny_edge canny_edge.out pics/$k.pgm 100 $i 100 >> ./deriv_$k.csv
	done

	for i in `seq 0 100`
	do
		~/powercycle.sh
		echo "Running gaussian $i $k"
		./canny_edge canny_edge.out pics/$k.pgm $i 100 100 >> ./gaus_$k.csv
	done
done