/* Protocol/process fixture ONLY. No simulation, Rust, Rapier or native steps.
 * All advertised counts are fabricated to exercise the launcher's parser.
 * Build: cc -std=c11 -O2 -pthread coordination-fake-worker.c -o fake-worker
 */
#define _POSIX_C_SOURCE 200809L
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

static atomic_int thread_started;
static void forever(void) { for (;;) pause(); }
static void *idle_thread(void *unused) {
    (void)unused;
    atomic_store(&thread_started, 1);
    forever();
    return NULL;
}
static void pid_record(void) {
    const char *path = getenv("LH_FAKE_PID_PATH");
    if (!path) _exit(80);
    int fd = open(path, O_WRONLY | O_APPEND | O_CREAT, 0600);
    if (fd < 0) _exit(81);
    char line[64];
    int n = snprintf(line, sizeof(line), "%ld\n", (long)getpid());
    if (write(fd, line, (size_t)n) != n || fsync(fd)) _exit(82);
    close(fd);
}
/* The fixture accepts only canonical simple JSON paths produced by our
 * controlled manifest. No external JSON library or subprocess is used. */
static int string_field(const char *json, const char *key, char *out, size_t cap) {
    char token[64];
    snprintf(token, sizeof(token), "\"%s\"", key);
    const char *p = strstr(json, token);
    if (!p || !(p = strchr(p + strlen(token), ':'))) return -1;
    while (isspace((unsigned char)*++p)) {}
    if (*p++ != '"') return -1;
    size_t n = 0;
    while (*p && *p != '"') {
        if (*p == '\\' || n + 1 >= cap) return -1;
        out[n++] = *p++;
    }
    if (*p != '"') return -1;
    out[n] = '\0';
    return 0;
}
static FILE *new_file(const char *path) {
    int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) return NULL;
    return fdopen(fd, "w");
}
static int durable_close(FILE *f) {
    if (fflush(f) || fsync(fileno(f))) return -1;
    return fclose(f);
}
int main(void) {
    const char *mode = getenv("LH_FAKE_WORKER_MODE");
    if (!mode) return 83;
    pid_record();
    if (!strcmp(mode, "crash-ready")) return 17;
    if (!strcmp(mode, "hang-ready")) forever();
    if (!strcmp(mode, "child")) {
        int ack[2];
        if (pipe(ack)) return 84;
        pid_t child = fork();
        if (child < 0) return 84;
        if (!child) {
            close(ack[0]); pid_record();
            if (write(ack[1], "x", 1) != 1) _exit(84);
            close(ack[1]); forever();
        }
        close(ack[1]);
        char ready;
        if (read(ack[0], &ready, 1) != 1) return 84;
        close(ack[0]);
    }
    if (!strcmp(mode, "two-threads")) {
        pthread_t thread;
        if (pthread_create(&thread, NULL, idle_thread, NULL)) return 85;
        while (!atomic_load(&thread_started)) sched_yield();
    }
    printf("{\"event\":\"ready\",\"pid\":%ld}\n", (long)getpid());
    fflush(stdout);
    char *request = NULL;
    size_t capacity = 0;
    if (getline(&request, &capacity, stdin) < 0) return 0;
    if (!strcmp(mode, "crash-permit")) return 18;
    if (!strcmp(mode, "hang") || !strcmp(mode, "child") || !strcmp(mode, "two-threads")) forever();
    char output[8192], trace[8192];
    if (string_field(request, "outputPath", output, sizeof(output)) ||
        string_field(request, "tracePath", trace, sizeof(trace))) return 86;
    free(request);
    int refused = !strcmp(mode, "refusal");
    int count = refused ? 12 : 44;
    const char *gates = refused
        ? "{\"integrity_ok\":true,\"prefix_matches_archive\":false,\"parity_verified\":false,\"diagnostic_present\":true}"
        : "{\"integrity_ok\":true,\"prefix_matches_archive\":true,\"parity_verified\":true,\"diagnostic_present\":true}";
    FILE *r = new_file(output), *t = new_file(trace);
    if (!r || !t) return 87;
    fprintf(r, "{\"actualNativeSteps\":%d,\"outcome\":%s,\"physicalIntegrations\":0,\"protocolCountsAreSynthetic\":true,\"fixture\":\"C process only; no simulation code\"}\n", count, gates);
    fprintf(t, "{\"physicalIntegrations\":0,\"protocolCountsAreSynthetic\":true,\"fabricatedProtocolSteps\":%d}\n", count);
    if (durable_close(r) || durable_close(t)) return 88;
    printf("{\"event\":\"result\",\"op\":\"numeric-parity\",\"actualNativeSteps\":%d,\"outcome\":%s,\"evidencePath\":\"%s\"}\n", count, gates, output);
    fflush(stdout);
    if (!strcmp(mode, "hang-eof")) forever();
    while (getchar() != EOF) {}
    return 0;
}
