#include "bufor.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


void init(buffer* buf)
{
	buf->possiblePlaces[0] = false;
	buf->possiblePlaces[1] = false;
	buf->possiblePlaces[2] = false;
}

void copyTable(uint8_t source[8], uint8_t destination[8])
{
	strncpy(destination, source, 8);
}

void addToBuffer(buffer* buf, uint8_t element[8])
{
	if (!isFull(buf))
	{
		if (!buf->possiblePlaces[0])
		{
			copyTable(element, buf->first);
			buf->possiblePlaces[0] = true;
		}
		else if (!buf->possiblePlaces[1])
		{
			copyTable(element, buf->second);
			buf->possiblePlaces[1] = true;
		}
		else if (!buf->possiblePlaces[2])
		{
			copyTable(element, buf->third);
			buf->possiblePlaces[2] = true;
		}
	}
}

uint8_t * removeFromBuffer(buffer* buf)
{
	if (!isEmpty(buf))
	{
		uint8_t returnedElement[8];
		copyTable(buf->first, returnedElement);
		buf->possiblePlaces[2] = false;
		if (buf->possiblePlaces[1])
		{
			copyTable(buf->second, buf->first);
				if (buf->possiblePlaces[2])
				{
					copyTable(buf->third, buf->second);

				}
				else
				{
					buf->possiblePlaces[1] = false;
				}
		}
		else
		{
			buf->possiblePlaces[0] = false;
		}
		return returnedElement;
	}
	else
		return NULL;
}

bool isEmpty(buffer* buf)
{
	if (buf->possiblePlaces[0] == false && buf->possiblePlaces[1] == false && buf->possiblePlaces[2] == false)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool isFull(buffer* buf)
{

	if (buf->possiblePlaces[0] == true && buf->possiblePlaces[1] == true && buf->possiblePlaces[2] == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}