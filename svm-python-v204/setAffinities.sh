#count=0;for i in `pgrep  svm_python_lear | xargs`; do taskset -p -c $count $i ;count=$((count+1)); done
count=0;for i in `ps | grep "svm_python" | cut -f 2 -d ' ' | xargs`; do taskset -p -c $count $i ;count=$((count+1)); done

