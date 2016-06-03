#!/bin/bash
for k in klomp tiger square
do
	for i in `seq 0 100`
	do
		~/powercycle.sh
		echo "Running magnitude $i $k"
		./canny_edge canny_edge.out pics/$k.pgm 100 100 $i >> $k_mag.csv
	done

	for i in `seq 0 100`
	do
		~/powercycle.sh
		echo "Running derivative $i $k"
		./canny_edge canny_edge.out pics/$k.pgm 100 $i 100 >> $k_deriv.csv
	done

	for i in `seq 0 100`
	do
		~/powercycle.sh
		echo "Running gaussian $i $k"
		./canny_edge canny_edge.out pics/$k.pgm $i 100 100 >> $k_gaus.csv
	done
done