#ifndef BUFOR_H
#define BUFOR_H

#include <stdint.h>
#include <stdbool.h>
typedef struct buffer
{
	uint8_t first[8];
	uint8_t second[8];
	uint8_t third[8];
	bool possiblePlaces[3];
	
} buffer;
	
	/* Funkcja inicjalizujace zmienne w buforze */
	void init(buffer*buf);

	/* Funkcja dodajaca nowy element do bufora */
	void addToBuffer(buffer* buf,uint8_t element);

	/* Funkcja usuwajaca z bufora element pierwszy(i jednoczesnie przesuwa pozostale) */
	uint8_t * removeFromBuffer(buffer* buf);

	/* Funkcja sprawdzajaca czy bufor jest pusty */
	bool isEmpty(buffer* buf);

	/* Funkcja sprawdzajaca czy bufor jest pelny */
	bool isFull(buffer* buf);

	/* Funkcja kopiujaca dane z jednej tablicy do drugiej */
	void copyTable(uint8_t source[8], uint8_t destination[8]);

#endif