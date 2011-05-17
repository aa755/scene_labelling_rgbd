sleep 2000
count=0;for i in `pgrep  svm_python_lear | xargs`; do taskset -p -c $count $i ;count=$((count+1)); done

