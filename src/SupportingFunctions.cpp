#include "SupportingFunctions.h"


SupportingFunctions::SupportingFunctions(void)
{
}


SupportingFunctions::~SupportingFunctions(void)
{
}

void SupportingFunctions::quicksort(vector<int> &arIntegers, int left, int right, vector<int> &list_followed)
{
    if (right > left)
    {
         int pivotIndex = median3(arIntegers,left,right, list_followed);
         int pivotNewIndex = partition(arIntegers, left, right, pivotIndex, list_followed);

         // Recursive call to quicksort to sort each half.
         quicksort(arIntegers, left, pivotNewIndex-1, list_followed);
         quicksort(arIntegers, pivotNewIndex+1, right, list_followed);
    }
}

int SupportingFunctions::median3(vector<int> &arIntegers,int left,int right, vector<int> &list_followed)
{
 	int center = (left+right)/2;

   if(arIntegers[center] < arIntegers[left])
   {
   	swap(arIntegers[left],arIntegers[center]);
	swap(list_followed[left], list_followed[center]);
   }
   if(arIntegers[right] < arIntegers[left])
   {
   	swap(arIntegers[left],arIntegers[right]);
	swap(list_followed[left],list_followed[right]);
   }
   if(arIntegers[right] < arIntegers[center])
   {
   	swap(arIntegers[center],arIntegers[right]);
	swap(list_followed[center],list_followed[right]);
   }

   swap(arIntegers[center],arIntegers[right-1]);
   swap(list_followed[center],list_followed[right-1]);

   return center;
}

// This function takes an array (or one half an array) and sorts it.
// It then returns a new pivot index number back to quicksort.

int SupportingFunctions::partition(vector<int> &arIntegers, int left, int right, int pivot, vector<int> &list_followed)
{
     int pivotValue = arIntegers[pivot];

     // Swap it out all the way to the end of the array
     // So we know where it always is.
     swap(arIntegers[pivot], arIntegers[right]);
	 swap(list_followed[pivot], list_followed[right]);
     int storeIndex = left;

     // Move through the array from start to finish comparing each to our
     // pivot value (not index, the value that was located at the pivot index)
     for (int i = left; i < right; i++)
     {
         if (arIntegers[i] <= pivotValue)
         {
             swap(arIntegers[i], arIntegers[storeIndex]);
			 swap(list_followed[i], list_followed[storeIndex]);
             storeIndex++;
         }
     }
     swap(arIntegers[storeIndex], arIntegers[right]);
	 swap(list_followed[storeIndex], list_followed[right]);
     return storeIndex;
}

// Simple swap function for our in place swapping.
void SupportingFunctions::swap(int &val1, int &val2)
{
    int temp = val1;
    val1 = val2;
    val2 = temp;
}