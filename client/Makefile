all:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tester.c -o tester.o
	gcc tester.o -Wall -lm -lev3dev-c -o tester
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c i2c.c -o i2c.o
	gcc i2c.o -Wall -lm -lev3dev-c -o i2c

i2c:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c i2c.c -o i2c.o
	gcc i2c.o -Wall -lm -lev3dev-c -o i2c
	./i2c

new:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c newTester.c -o newTester.o
	gcc newTester.o -Wall -lm -lev3dev-c -o newTester

tester:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c tester.c -o tester.o
	gcc tester.o -Wall -lm -lev3dev-c -o tester
	./tester

coroutin: 
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c coroutin.c -o coroutin.o
	gcc coroutin.o -Wall -lm -lev3dev-c -o coroutin

twc:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c testerWithClass.c -o testerWithClass.o
	gcc testerWithClass.o -Wall -lm -lev3dev-c -o testerWithClass

ptest:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c parallelTest.c -o parallelTest.o
	gcc parallelTest.o -Wall -lm -lev3dev-c -o parallelTest

retry:
	rm -f retry.o retry
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c retry.c -o retry.o
	gcc retry.o -Wall -lm -lev3dev-c -o retry

jtest:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c johanneTest.c -o johanneTest.o
	gcc johanneTest.o -Wall -lm -lev3dev-c -o johanneTest
