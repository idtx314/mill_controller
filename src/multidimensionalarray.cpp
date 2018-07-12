#include<iostream>
#include<cstdio>


int main(void)
{
    int arr[2][2][2] = {0,1,2,3,4,5,6,7};

    for(int z=0; z<2; z++) //Cycle through depths
    {
        for(int y=0; y<2; y++) //Cycle through rows
        {
            for(int x=0; x<2; x++) //Cycle through columns
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
