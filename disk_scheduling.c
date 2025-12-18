/***************************************************************
 * A4Q1.c
 * COIS 3380 — Disk Scheduling Algorithms
 *
 * Implements classical disk scheduling policies on a
 * 300-cylinder disk (0–299) using 20 requests loaded from a
 * binary input file (request.bin).
 *
 * Algorithms implemented:
 *   • FCFS
 *   • SSTF
 *   • SCAN
 *   • C-SCAN
 *   • LOOK
 *   • C-LOOK
 *
 * The program accepts two command-line parameters:
 *   1) Initial head position
 *   2) Scan direction: LEFT or RIGHT
 *
 * Output:
 *   • Actual service order for each algorithm
 *   • Total head movement (in cylinders)
 *
 * Design:
 *   Algorithms producing monotonic sweeps (SCAN, LOOK variants)
 *   operate on a sorted request array. FCFS and SSTF use the
 *   original ordering. Head movement is computed generically.
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define NUM_CYLINDERS 300
#define NUM_REQUESTS  20

typedef enum { DIR_LEFT, DIR_RIGHT } Direction;

/****************************************************************
 * parse_direction
 * Converts a direction argument into an enum.
 * Exits immediately if the argument is invalid.
 ****************************************************************/
Direction parse_direction(const char *s) {
    if (strcmp(s, "LEFT") == 0) return DIR_LEFT;
    if (strcmp(s, "RIGHT") == 0) return DIR_RIGHT;

    fprintf(stderr, "ERROR: Direction must be LEFT or RIGHT.\n");
    exit(1);
}

/****************************************************************
 * cmp_int
 * qsort comparator for ascending integer order.
 ****************************************************************/
int cmp_int(const void *a, const void *b) {
    int x = *(const int *)a;
    int y = *(const int *)b;
    return (x > y) - (x < y);
}

/****************************************************************
 * compute_movement
 * Computes cumulative head movement given a service sequence.
 ****************************************************************/
int compute_movement(int seq[], int len, int start) {
    int head = start;
    int total = 0;

    for (int i = 0; i < len; i++) {
        total += abs(seq[i] - head);
        head = seq[i];
    }
    return total;
}

/****************************************************************
 * Struct representing the result of a scheduling algorithm:
 *   - seq: serviced request order
 *   - len: number of items in seq
 *   - movement: total head movement
 ****************************************************************/
typedef struct {
    int seq[50];   // extra capacity for SCAN/C-SCAN endpoints
    int len;
    int movement;
} Result;

/****************************************************************
 * FCFS - First Come First Served
 * Processes requests strictly in arrival order.
 ****************************************************************/
Result schedule_fcfs(int req[], int start) {
    Result r;
    r.len = NUM_REQUESTS;

    for (int i = 0; i < NUM_REQUESTS; i++)
        r.seq[i] = req[i];

    r.movement = compute_movement(r.seq, r.len, start);
    return r;
}

/****************************************************************
 * SSTF - Shortest Seek Time First
 * Greedily selects the nearest unserviced request.
 ****************************************************************/
Result schedule_sstf(int req[], int start) {
    Result r;
    int visited[NUM_REQUESTS] = {0};
    int head = start;
    int count = 0;

    while (count < NUM_REQUESTS) {
        int bestDist = 1e9;
        int bestIdx = -1;

        for (int i = 0; i < NUM_REQUESTS; i++) {
            if (!visited[i]) {
                int d = abs(req[i] - head);
                if (d < bestDist) {
                    bestDist = d;
                    bestIdx = i;
                }
            }
        }

        visited[bestIdx] = 1;
        r.seq[count++] = req[bestIdx];
        head = req[bestIdx];
    }

    r.len = NUM_REQUESTS;
    r.movement = compute_movement(r.seq, r.len, start);
    return r;
}

/****************************************************************
 * find_index
 * Locates the first sorted request >= start.
 ****************************************************************/
int find_index(int sorted[], int n, int start) {
    for (int i = 0; i < n; i++)
        if (sorted[i] >= start)
            return i;
    return n;  // all requests are smaller
}

/****************************************************************
 * SCAN - "Elevator Algorithm"
 * Performs a monotonic sweep in the initial direction until
 * reaching the physical boundary, then reverses.
 ****************************************************************/
Result schedule_scan(int sorted[], int start, Direction dir) {
    Result r;
    r.len = 0;

    int idx = find_index(sorted, NUM_REQUESTS, start);

    if (dir == DIR_LEFT) {
        for (int i = idx - 1; i >= 0; i--)
            r.seq[r.len++] = sorted[i];

        r.seq[r.len++] = 0;  // physical boundary

        for (int i = idx; i < NUM_REQUESTS; i++)
            r.seq[r.len++] = sorted[i];
    }
    else { // DIR_RIGHT
        for (int i = idx; i < NUM_REQUESTS; i++)
            r.seq[r.len++] = sorted[i];

        r.seq[r.len++] = NUM_CYLINDERS - 1; // boundary

        for (int i = idx - 1; i >= 0; i--)
            r.seq[r.len++] = sorted[i];
    }

    r.movement = compute_movement(r.seq, r.len, start);
    return r;
}

/****************************************************************
 * C-SCAN - Circular SCAN
 * Monotonic sweep in one direction only. Upon reaching the
 * boundary, wraps directly to the opposite end and continues.
 ****************************************************************/
Result schedule_cscan(int sorted[], int start, Direction dir) {
    Result r;
    r.len = 0;
    int idx = find_index(sorted, NUM_REQUESTS, start);

    if (dir == DIR_RIGHT) {
        for (int i = idx; i < NUM_REQUESTS; i++)
            r.seq[r.len++] = sorted[i];

        r.seq[r.len++] = NUM_CYLINDERS - 1; // right boundary
        r.seq[r.len++] = 0;                 // wrap-around

        for (int i = 0; i < idx; i++)
            r.seq[r.len++] = sorted[i];
    }
    else { // DIR_LEFT
        for (int i = idx - 1; i >= 0; i--)
            r.seq[r.len++] = sorted[i];

        r.seq[r.len++] = 0;                 // left boundary
        r.seq[r.len++] = NUM_CYLINDERS - 1; // wrap-around

        for (int i = NUM_REQUESTS - 1; i >= idx; i--)
            r.seq[r.len++] = sorted[i];
    }

    r.movement = compute_movement(r.seq, r.len, start);
    return r;
}

/****************************************************************
 * LOOK
 * Like SCAN, but does not travel to physical boundaries unless
 * required by actual requests. Only scans as far as needed.
 ****************************************************************/
Result schedule_look(int sorted[], int start, Direction dir) {
    Result r;
    r.len = 0;
    int idx = find_index(sorted, NUM_REQUESTS, start);

    if (dir == DIR_LEFT) {
        for (int i = idx - 1; i >= 0; i--)
            r.seq[r.len++] = sorted[i];

        for (int i = idx; i < NUM_REQUESTS; i++)
            r.seq[r.len++] = sorted[i];
    }
    else { // DIR_RIGHT
        for (int i = idx; i < NUM_REQUESTS; i++)
            r.seq[r.len++] = sorted[i];

        for (int i = idx - 1; i >= 0; i--)
            r.seq[r.len++] = sorted[i];
    }

    r.movement = compute_movement(r.seq, r.len, start);
    return r;
}

/****************************************************************
 * C-LOOK
 * Circular version of LOOK. Wraps from one end of the request
 * list to the other without touching unused physical cylinders.
 ****************************************************************/
Result schedule_clook(int sorted[], int start, Direction dir) {
    Result r;
    r.len = 0;
    int idx = find_index(sorted, NUM_REQUESTS, start);

    if (dir == DIR_RIGHT) {
        for (int i = idx; i < NUM_REQUESTS; i++)
            r.seq[r.len++] = sorted[i];

        for (int i = 0; i < idx; i++)
            r.seq[r.len++] = sorted[i];
    }
    else { // DIR_LEFT
        for (int i = idx - 1; i >= 0; i--)
            r.seq[r.len++] = sorted[i];

        for (int i = NUM_REQUESTS - 1; i >= idx; i--)
            r.seq[r.len++] = sorted[i];
    }

    r.movement = compute_movement(r.seq, r.len, start);
    return r;
}

/****************************************************************
 * print_result
 * Prints sequence and total movement in the required format.
 ****************************************************************/
void print_result(const char *name, Result r) {
    printf("%s DISK SCHEDULING ALGORITHM:\n\n", name);

    for (int i = 0; i < r.len; i++) {
        printf("%d", r.seq[i]);
        if (i < r.len - 1) printf(", ");
    }

    printf("\n\n%s - Total head movements = %d\n\n",
           name, r.movement);
}

/****************************************************************
 * main
 * Coordinates:
 *    argument parsing
 *    file I/O for request.bin
 *    sorting
 *    invocation of all algorithms
 ****************************************************************/
int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: ./A4Q1 <initial> <LEFT|RIGHT>\n");
        return 1;
    }

    int start = atoi(argv[1]);
    if (start < 0 || start >= NUM_CYLINDERS) {
        fprintf(stderr, "ERROR: Initial head must be between 0 and 299.\n");
        return 1;
    }

    Direction dir = parse_direction(argv[2]);

    int req[NUM_REQUESTS];
    FILE *fp = fopen("request.bin", "rb");
    if (!fp) {
        perror("ERROR opening request.bin");
        return 1;
    }

    if (fread(req, sizeof(int), NUM_REQUESTS, fp) != NUM_REQUESTS) {
        fprintf(stderr, "ERROR: Could not read all requests.\n");
        return 1;
    }
    fclose(fp);

    int sorted[NUM_REQUESTS];
    memcpy(sorted, req, sizeof(req));
    qsort(sorted, NUM_REQUESTS, sizeof(int), cmp_int);

    printf("Total requests = %d\n", NUM_REQUESTS);
    printf("Initial Head Position: %d\n", start);
    printf("Direction of Head: %s\n\n",
           dir == DIR_LEFT ? "LEFT" : "RIGHT");

    print_result("FCFS",   schedule_fcfs(req, start));
    print_result("SSTF",   schedule_sstf(req, start));
    print_result("SCAN",   schedule_scan(sorted, start, dir));
    print_result("C-SCAN", schedule_cscan(sorted, start, dir));
    print_result("LOOK",   schedule_look(sorted, start, dir));
    print_result("C-LOOK", schedule_clook(sorted, start, dir));

    return 0;
}

