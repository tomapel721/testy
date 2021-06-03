#include <stdio.h>
#include "bufor.h"

int main()
{
	// testowanie
	buffer myBuffer;
	isFull(&myBuffer);
	init(&myBuffer);

	uint8_t tab[8];
	for (int i = 0; i < 8; i++)
	{
		tab[i] = i+15;
	}
	printf("%d\n", *tab);
	uint8_t tab0[8];
	for (int i = 0; i < 8; i++)
	{
		tab0[i] = 12+i;
	}
	printf("%d\n\n", *tab0);
	addToBuffer(&myBuffer, tab0);
	addToBuffer(&myBuffer, tab);
	uint8_t nowa[8];
	copyTable(removeFromBuffer(&myBuffer), nowa);
	
	for (int i = 0; i < 8; i++)
	{
		printf("%d\n", nowa[i]);
	}

}