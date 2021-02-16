out: main.cpp
	g++ -o out main.cpp -pthread -l bcm2835

dac: dac.cpp
	g++ -o dac dac.cpp -l bcm2835

clean:
	rm *.o out -rf
