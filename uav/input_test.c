#include <stdio.h>

int main(void)
{
    int a;
    char b;
    int running = 1;

    while (running)
    {
        printf("Enter an int: ");
        scanf("%d", &a);
        printf("\nYou entered: ");
        printf("%d", a);
        printf("\n");

        printf("Enter a char: ");
        scanf(" %c", &b);
        if (b == 'q')
        {
            return 0;
        }
        else if (b == 'e')
        {
            // fflush(stdout);
        }
        else
        {
            printf("You entered: ");
            printf("%c", b);
            printf("\n");
        }
    }
}
