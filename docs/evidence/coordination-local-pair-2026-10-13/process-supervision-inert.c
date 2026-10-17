/* Inert OS-supervision fixture. No model, physics, integration or native worker. */
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
static void *idle(void *unused) { (void)unused; for (;;) pause(); return NULL; }
int main(void) {
    const char *mode = getenv("LH_INERT_MODE");
    if (!mode) mode = "normal";
    if (!strcmp(mode, "thread")) { pthread_t t; if (pthread_create(&t, NULL, idle, NULL)) return 3; }
    if (!strcmp(mode, "child")) { pid_t child = fork(); if (child < 0) return 4; if (!child) for (;;) pause(); }
    printf("{\"event\":\"ready\",\"pid\":%d}\n", (int)getpid()); fflush(stdout);
    if (!strcmp(mode, "hang")) for (;;) pause();
    char line[256]; while (fgets(line, sizeof(line), stdin)) { /* No instruction executes anything. */ }
    return 0;
}
