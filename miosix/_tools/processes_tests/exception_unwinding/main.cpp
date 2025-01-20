#include <cstdio>

void __attribute__((noinline)) f()
{
   throw 1;
}

int main() try {
    puts("in");
    f();
    puts("out");
    return 0;
} catch(int& e) {
    puts("exc");
    return e;
}
