// main.c
#include <stdio.h>
#include <dlfcn.h>  // On Linux, use <windows.h> on Windows

// Declare the function prototype
void myFunction();

int main() {
    // Load the shared library
    void *handle = dlopen("./libtest.so", RTLD_LAZY);  // Use "myfunctions.dll" on Windows

    if (!handle) {
        fprintf(stderr, "%s\n", dlerror());
        return 1;
    }

    // Get a pointer to the function
    void (*func)() = dlsym(handle, "myFunction");

    if (dlerror() != NULL) {
        fprintf(stderr, "%s\n", dlerror());
        return 1;
    }

    // Call the function
    func();

    // Close the shared library
    dlclose(handle);

    return 0;
}
