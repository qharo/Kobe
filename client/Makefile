all:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c defence.c -o defence.o
	gcc defence.o -Wall -lm -lev3dev-c -o defence
	gcc -I./ev3dev-c/source/ev3 -O0 -std=gnu99 -W -Wall -Wno-comment -c attack.c -o attack.o
	gcc attack.o -O0 -Wall -lm -lev3dev-c -o attack
	
def:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c defence.c -o defence.o
	gcc defence.o -Wall -lm -lev3dev-c -o defence

attack: attack.c
	gcc -I./ev3dev-c/source/ev3 -O0 -std=gnu99 -W -Wall -Wno-comment -c attack.c -o attack.o
	gcc attack.o -O0 -Wall -lm -lev3dev-c -o attack

