#!/bin/bash

instance_dir=instances
methods=( "mtz" "scf" "mcf" )

for file in $instance_dir/*.prob
do
  for method in "${methods[@]}"
  do
    filename=`basename $file`
    echo "qsub -N tcbvrp_${method}_${filename} -o ${filename}_${method}.out -e ${filename}_${method}.err ./script.sh -f $file -m $method"
  done
done
