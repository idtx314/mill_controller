#include<iostream>
#include<cstdio>

#define DEPT 2

// Usage $ ./multi_dimensional_array.exe Columns Rows Depth

int main(int argc, char** argv)
{
    if(argc != 4)
        return 1;

    // Define and initialize variables
    int COL,ROW,DEP;
    char counter= 0;

    // Save arguments into variables
    sscanf(argv[1], "%d", &COL);
    sscanf(argv[2], "%d", &ROW);
    sscanf(argv[3], "%d", &DEP);

    // std::cout << (char)COL << " " << (char)ROW << " " << (char)DEP << std::endl;

    // Echo input
    printf("%d %d %d\n",COL, ROW, DEP);

    // Define array (can also initialize here if contents are known)
    // int arr[DEP][ROW][COL] = {{{0,1},{2,3}},{{4,5},{6,7}}};
    char arr[DEP][ROW][COL];

    // Initialize array
    for(int z=0; z<DEP; z++) //Cycle through depths
    {
        for(int y=0; y<ROW; y++) //Cycle through rows
        {
            for(int x=0; x<COL; x++) //Cycle through columns
            {
                arr[z][y][x] = counter;
                if(counter==0)
                    counter=1;
                else
                    counter=0;
            }
        }
    }

    // Print array
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
