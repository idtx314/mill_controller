#include<iostream>
#include<cstdio>

#define DEPT 2

// Usage $ ./multi_dimensional_array.exe Columns Rows Depth

int main(int argc, char** argv)
{
    if(argc != 4)
        return 1;

    int COL,ROW,DEP;

    sscanf(argv[1], "%d", &COL);
    sscanf(argv[2], "%d", &ROW);
    sscanf(argv[3], "%d", &DEP);

    // std::cout << (char)COL << " " << (char)ROW << " " << (char)DEP << std::endl;

    printf("%d %d %d\n",COL, ROW, DEP);

    int arr[DEPT][ROW][COL] = {0,1,2,3,4,5,6,7};

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
