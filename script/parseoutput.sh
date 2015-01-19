#!/bin/bash

instance_dir=instances
outdir=output
methods=( "mtz" "scf" "mcf" )

sizes=( 10 20 30 60 90 120 180 )
indexes=( 1 2 )
limits=( 240 360 480 720 )
ms=( 2 3 4 6 8 10 )


for method in "${methods[@]}"
do
    
    echo
    echo $method
    
    for size in "${sizes[@]}"
    do 
	for index in "${indexes[@]}"
	do 
	    for limit in "${limits[@]}"
	    do 
		for m in "${ms[@]}"
		do 
		    filename="tcbvrp_${size}_${index}_T${limit}_m${m}.prob"
		    outfile="${outdir}/${filename}_${method}.out"
		    errfile="${outdir}/${filename}_${method}.err"

		    if [ -e "${outfile}" ]
		    then 
			
			status=`grep "CPLEX status" $outfile | awk '{print $3;}'`
			bnb=`grep "Branch-and-Bound" $outfile | awk '{print $3;}'`
			time=`grep "CPU time" $outfile | awk '{print $3;}'`
			objective=`grep "Objective value" $outfile | awk '{print $3;}'`
			
			echo "${size}_${index}_T${limit}_m${m} & ${status} & ${objective} & ${bnb} & ${time} \\\\"
		    fi
		done
	    done
	done
    done    
done
