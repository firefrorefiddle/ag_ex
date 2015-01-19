#!/bin/bash

instance_dir=instances
output_dir=output
methods=( "mtz" "scf" "mcf" )

for file in $instance_dir/*.prob
do
  for method in "${methods[@]}"
  do
    filename=`basename $file`
    outfile="${output_dir}/${filename}_${method}.out"
    errfile="${output_dir}/${filename}_${method}.err"
    echo "rm -f ${outfile}"
    echo "rm -f ${errfile}"
    echo "qsub -N tcbvrp_${method}_${filename} -o ${outfile} -e ${errfile} ./script.sh -f $file -m $method"
  done
done
