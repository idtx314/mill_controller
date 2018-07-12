#include<iostream>
#include<cstdio>


// Usage $ ./multi_dimensional_array.exe Columns Rows Depth

int main(int argc, char** argv)
{
    int arr[DEP][ROW][COL] = {0,1,2,3,4,5,6,7};

    if(argc != 4)
        return 1;

    int COL = *argv[1];
    int ROW = *argv[2];
    int DEP = *argv[3];

    for(int z=0; z<DEP; z++) //Cycle through depths
    {
        for(int y=0; y<ROW; y++) //Cycle through rows
        {
            for(int x=0; x<COL; x++) //Cycle through columns
            {
                printf("%d ",arr[z][y][x]);
            }
            printf("\n");
        }
        printf("\n");
    }

    char holder;
    std::cin >> holder;

    return 0;
}
