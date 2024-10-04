#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FALSE (0)
#define TRUE (!FALSE)

typedef struct {
	int32_t data;
	uint32_t idx;
}ST_MedianData;

typedef struct {

	ST_MedianData* buffer;
	uint32_t length;
	uint32_t currentIdx;
	uint32_t centerIdx;
}ST_MedianFilter;

int32_t MedianFilterInit(ST_MedianFilter* medianFilter, uint32_t length)
{
	int32_t i;
	int32_t result = TRUE;

	medianFilter->length = length;
	medianFilter->centerIdx = length / 2;
	medianFilter->currentIdx = 0;

	medianFilter->buffer = (ST_MedianData*)malloc(length * sizeof(ST_MedianData));
	if (medianFilter->buffer == NULL) {
		result = FALSE;
	}
	else {
		for (i = 0; i < length; i++) {
			medianFilter->buffer[i].data = 0;
			medianFilter->buffer[i].idx = i;
		}
	}

	return result;
}

void MedianFilterDelete(ST_MedianFilter* medianFilter)
{
	free(medianFilter->buffer);
}

int32_t Compare(const void* first, const void* second)
{
	int32_t result;

	if (((ST_MedianData*)first)->data > ((ST_MedianData*)second)->data) {
		result = 1;
	}
	else if (((ST_MedianData*)first)->data < ((ST_MedianData*)second)->data) {
		result = -1;
	}
	else {
		result = 0;
	}

	return result;
}

int32_t MedianFilter(ST_MedianFilter* medianFilter, int32_t value)
{
	int32_t i;
	int32_t result;

	for (i = 0; i < medianFilter->length; i++) {
		if (medianFilter->buffer[i].idx == medianFilter->currentIdx) {
			medianFilter->buffer[i].data = value;
		}
	}

	medianFilter->currentIdx++;
	medianFilter->currentIdx %= medianFilter->length;

	qsort(medianFilter->buffer, medianFilter->length, sizeof(medianFilter->buffer[0]), Compare);

	result = medianFilter->buffer[medianFilter->centerIdx].data;

	return result;
}

typedef struct ST_Node{
	int32_t data;
	int32_t idx;
	struct ST_Node* next;
	struct ST_Node* prev;
}ST_Node;

typedef struct {
	ST_Node* head;
	size_t size;
	size_t filterSize;
	int32_t currentIdx;
}ST_MedianFilterList;

int32_t MedianFilterListInit(ST_MedianFilterList** list, size_t filterSize)
{
	int32_t result;

	*list = (ST_MedianFilterList*)malloc(sizeof(ST_MedianFilterList));

	if (list == NULL) {
		result = FALSE;
	}
	else {
		(*list)->head = NULL;
		(*list)->size = 0;
		(*list)->filterSize = filterSize;
		(*list)->currentIdx = 0;
		result = TRUE;
	}
	
	return result;
}

void MedianFilterListDelete(ST_MedianFilterList* list)
{
	ST_Node* temp;
	ST_Node* deleteNode;

	temp = list->head;

	while (temp != NULL) {
		deleteNode = temp;
		temp = temp->next;
		free(deleteNode);
	}

	free(list);
}

int32_t MedianFilterListPush(ST_MedianFilterList* list, int32_t data, int32_t idx)
{
	int32_t result = TRUE;
	ST_Node* add;
	ST_Node* temp;

	do {
		add = (ST_Node*)malloc(sizeof(ST_Node));
		if (add == NULL) {
			result = FALSE;
			break;
		}

		add->data = data;
		add->idx = idx;
		add->next = add->prev = NULL;
		list->size++;

		if (list->head == NULL) {
			list->head = add;
			break;
		}

		temp = list->head;
		if (add->data <= temp->data) {
			add->next = temp;
			temp->prev = add;
			list->head = add;
			break;
		}

		while (temp->next != NULL) {
			if (temp->data <= add->data && add->data <= temp->next->data) {
				temp->next->prev = add;
				add->next = temp->next;
				temp->next = add;
				add->prev = temp;
				break;
			}

			temp = temp->next;
		}

		if (temp->next == NULL) {
			temp->next = add;
			add->prev = temp;
		}

	} while (0);
	
	return result;
}

int32_t MedianFilterListPop(ST_MedianFilterList* list, int32_t idx)
{
	int32_t result = FALSE;
	ST_Node* temp;

	temp = list->head;
	while (temp != NULL) {
		if (temp->idx == idx) {
			if (temp->prev == NULL) {
				temp->next->prev = NULL;
				list->head = temp->next;
			}
			else if (temp->next == NULL) {
				temp->prev->next = NULL;
			}
			else {
				temp->prev->next = temp->next;
				temp->next->prev = temp->prev;
			}
			free(temp);
			list->size--;

			result = TRUE;
			break;
		}

		temp = temp->next;
	}
	
	return result;
}

int32_t MedianFilterListAt(ST_MedianFilterList* list, int32_t loop) {
	ST_Node* temp;

	temp = list->head;
	for (int32_t i = 0; i < loop; i++) {
		temp = temp->next;
	}

	return temp->data;
}

void printList(ST_MedianFilterList* list)
{
	ST_Node* temp;

	temp = list->head;
	while(temp != NULL){
		printf("%d ", temp->data);
		temp = temp->next;
	}
	printf("\n");

	temp = list->head;
	while (temp != NULL) {
		printf("%d ", temp->idx);
		temp = temp->next;
	}
	printf("\n");
}

int32_t medianFilterList(ST_MedianFilterList* list, int32_t data)
{
	int32_t result;

	if (list->size == list->filterSize) {
		MedianFilterListPop(list, list->currentIdx);
	}
	MedianFilterListPush(list, data, list->currentIdx);
	result = MedianFilterListAt(list, (list->size / 2));

	list->currentIdx++;
	list->currentIdx %= list->filterSize;

	return result;
}

#define LOOP_COUNT (10000)

int32_t sensorOutput[LOOP_COUNT];

int main()
{
	ST_MedianFilterList* list = NULL;
	ST_MedianFilter medianFilter;
	int32_t filter;
	clock_t start[2], end[2];
	double result[2];
	
	if (MedianFilterListInit(&list, 64) == FALSE) {
		return 0;
	}

	if (MedianFilterInit(&medianFilter, 64) == FALSE) {
		return 0;
	}

	for (int32_t i = 0; i < LOOP_COUNT; i++) {
		sensorOutput[i] = 50 + (rand() % 21 - 10);
	}
	
	start[0] = clock();
	for (int32_t i = 0; i < LOOP_COUNT; i++) {
		filter = MedianFilter(&medianFilter, sensorOutput[i]);
	}
	end[0] = clock();


	start[1] = clock();
	for (int32_t i = 0; i < LOOP_COUNT; i++) {
		filter = medianFilterList(list, sensorOutput[i]);
	}
	end[1] = clock();

	printf("start[0]=%d, end[0]=%d, start[1]=%d, end[1]=%d\n", start[0], end[0], start[1], end[1]);
	
	result[0] = double(end[0] - start[0]);
	result[1] = double(end[1] - start[1]);

	printf("%lf, %lf\n", result[0], result[1]);

	MedianFilterListDelete(list);
	MedianFilterDelete(&medianFilter);

	return 0;
}