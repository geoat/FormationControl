g++ -std=c++11 -o a.out -I. Node0.cpp
./a.out
gnuplot<<- EOF 
set samples 10000
set term png
set xlabel "x"
set ylabel "y"
set output "output_plot.png"
plot "virtualdrone_output.Csv" using 1:2 title "Virtual Drone" w lines, "pathplanner_output.Csv" using 1:2 title "Path Planner" with lines
EOF