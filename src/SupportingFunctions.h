#include<iostream>
#include<vector>

using namespace std;

class SupportingFunctions
{
public:
	SupportingFunctions(void);
	~SupportingFunctions(void);

	static void quicksort(vector<int> &, int, int, vector<int> &);
	static int partition(vector<int> &, int, int, int, vector<int> &);
	static int median3(vector<int> &,int,int, vector<int> &);
	static void swap(int &, int &);
};
