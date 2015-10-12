#!/bin/sh

file="running_proc_id.txt"

if [ -f "$file" ]
then
	while read -r line; do
		echo "stopping $line"
    	kill "$line"
	done < "$file"
fi

rm $file

