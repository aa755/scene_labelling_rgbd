rm allTimes
rm LPTimes
rm MIPTimes
for i in `seq 1 4` 
do
echo "fold$i "
grep "Time for "  fold$i/pred/$1 >> allTimes
grep "Time for LP"  allTimes >> LPtimes
grep "Time for MIP"  allTimes >> MIPtimes

done

echo "LP stats"
cut -f 2 -d ':' LPtimes | sort -n | awk '{ s += $1 } END { print "sum: ", s, " average: ", s/NR, " samples: ", NR }'
echo "MIP stats"
cut -f 2 -d ':' MIPtimes | sort -n > MIPTimes.clean
cat MIPTimes.clean | awk '{ s += $1 } END { print "sum: ", s, " average: ", s/NR, " samples: ", NR }'
