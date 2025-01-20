extern const int aRodata;
extern int aData;
const char str[] = "Hello world\n";

int get1() { return aRodata; }

int get2() { return aData; }

const char *get3() { return str; }

int main()
{
    return 0;
}
