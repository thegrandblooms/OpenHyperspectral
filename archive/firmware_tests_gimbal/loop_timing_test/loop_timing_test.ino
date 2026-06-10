/**
 * Loop Timing Test
 *
 * This test verifies that the main control loop maintains a stable 1000 Hz
 * frequency as required for FOC motor control. Measures loop timing accuracy,
 * jitter, and CPU performance.
 *
 * Test Objectives:
 * - Verify 1000 Hz (1000 μs period) loop timing
 * - Measure timing jitter and consistency
 * - Monitor CPU utilization
 * - Detect timing violations
 * - Profile loop execution time
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Open Serial Monitor at 115200 baud
 * 3. Observe timing statistics
 * 4. Send 'r' to reset statistics, 's' to show detailed stats
 */

#include <Arduino.h>

// Loop configuration (from config.h)
#define LOOP_FREQUENCY_HZ 1000          // Target loop frequency
#define LOOP_PERIOD_US    (1000000 / LOOP_FREQUENCY_HZ)  // 1000 μs
#define HEARTBEAT_INTERVAL 10000  // 10 seconds

// Statistics tracking
unsigned long loop_count = 0;
unsigned long timing_violations = 0;
unsigned long total_loop_time_us = 0;
unsigned long max_loop_time_us = 0;
unsigned long min_loop_time_us = UINT32_MAX;

unsigned long last_update_time = 0;
unsigned long last_heartbeat = 0;

// Detailed statistics (sliding window)
#define STATS_WINDOW_SIZE 1000
unsigned long timing_histogram[10] = {0};  // Timing distribution buckets
float jitter_sum = 0;
unsigned long jitter_samples = 0;

String command_buffer = "";

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  LOOP TIMING TEST - COMMANDS                   ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("Commands:");
    Serial.println("  h, help     - Show this help menu");
    Serial.println("  s, stats    - Show detailed statistics");
    Serial.println("  r, reset    - Reset statistics");
    Serial.println("  d, dist     - Show timing distribution");
    Serial.println("  i, info     - Show system information");
    Serial.println();
}

void printSystemInfo() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   SYSTEM INFORMATION                           ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("CPU Frequency: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println(" MB");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.println();
}

void printStatistics() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  LOOP TIMING STATISTICS                        ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    Serial.print("Target Loop Period: ");
    Serial.print(LOOP_PERIOD_US);
    Serial.println(" μs (1000 Hz)");

    Serial.print("Total Loops: ");
    Serial.println(loop_count);

    Serial.print("Timing Violations: ");
    Serial.print(timing_violations);
    if (loop_count > 0) {
        float violation_rate = (float)timing_violations / loop_count * 100.0;
        Serial.print(" (");
        Serial.print(violation_rate, 2);
        Serial.println("%)");
    } else {
        Serial.println();
    }

    if (loop_count > 0) {
        unsigned long avg_loop_time = total_loop_time_us / loop_count;
        Serial.print("Average Loop Time: ");
        Serial.print(avg_loop_time);
        Serial.println(" μs");

        Serial.print("Min Loop Time: ");
        Serial.print(min_loop_time_us);
        Serial.println(" μs");

        Serial.print("Max Loop Time: ");
        Serial.print(max_loop_time_us);
        Serial.println(" μs");

        if (jitter_samples > 0) {
            float avg_jitter = jitter_sum / jitter_samples;
            Serial.print("Average Jitter: ");
            Serial.print(avg_jitter, 2);
            Serial.println(" μs");
        }

        // Calculate CPU utilization
        float cpu_usage = (float)avg_loop_time / LOOP_PERIOD_US * 100.0;
        Serial.print("CPU Utilization: ");
        Serial.print(cpu_usage, 2);
        Serial.println("%");

        // Calculate headroom
        float headroom = LOOP_PERIOD_US - avg_loop_time;
        Serial.print("Timing Headroom: ");
        Serial.print(headroom, 0);
        Serial.print(" μs (");
        Serial.print(headroom / LOOP_PERIOD_US * 100.0, 1);
        Serial.println("%)");
    }

    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.println();
}

void printDistribution() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  TIMING DISTRIBUTION                           ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Loop time distribution (μs):");

    const char* ranges[] = {
        "  0-100 μs  : ",
        "101-200 μs  : ",
        "201-400 μs  : ",
        "401-600 μs  : ",
        "601-800 μs  : ",
        "801-1000 μs : ",
        "1001-1200 μs: ",
        "1201-1500 μs: ",
        "1501-2000 μs: ",
        ">2000 μs    : "
    };

    unsigned long max_count = 0;
    for (int i = 0; i < 10; i++) {
        if (timing_histogram[i] > max_count) {
            max_count = timing_histogram[i];
        }
    }

    for (int i = 0; i < 10; i++) {
        Serial.print(ranges[i]);
        Serial.print(timing_histogram[i]);

        // Draw bar graph
        if (max_count > 0) {
            Serial.print(" |");
            int bar_length = (timing_histogram[i] * 40) / max_count;
            for (int j = 0; j < bar_length; j++) {
                Serial.print("█");
            }
        }
        Serial.println();
    }
    Serial.println();
}

void resetStatistics() {
    loop_count = 0;
    timing_violations = 0;
    total_loop_time_us = 0;
    max_loop_time_us = 0;
    min_loop_time_us = UINT32_MAX;
    jitter_sum = 0;
    jitter_samples = 0;

    for (int i = 0; i < 10; i++) {
        timing_histogram[i] = 0;
    }

    Serial.println("Statistics reset!");
}

void processCommand() {
    command_buffer.trim();
    command_buffer.toLowerCase();

    if (command_buffer.length() == 0) return;

    if (command_buffer == "h" || command_buffer == "help") {
        printHelp();
    }
    else if (command_buffer == "s" || command_buffer == "stats") {
        printStatistics();
    }
    else if (command_buffer == "r" || command_buffer == "reset") {
        resetStatistics();
    }
    else if (command_buffer == "d" || command_buffer == "dist") {
        printDistribution();
    }
    else if (command_buffer == "i" || command_buffer == "info") {
        printSystemInfo();
    }
    else {
        Serial.print("Unknown command: '");
        Serial.print(command_buffer);
        Serial.println("'. Type 'h' for help.");
    }

    command_buffer = "";
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  LOOP TIMING TEST                              ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Test Configuration:");
    Serial.print("  - Target Frequency: ");
    Serial.print(LOOP_FREQUENCY_HZ);
    Serial.println(" Hz");
    Serial.print("  - Target Period: ");
    Serial.print(LOOP_PERIOD_US);
    Serial.println(" μs");
    Serial.print("  - CPU Frequency: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.println();

    Serial.println("[INIT] Starting timing test...");
    Serial.println("[OK]   Test initialized");
    Serial.println();

    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   MONITORING LOOP TIMING...                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("The test will continuously monitor loop timing accuracy.");
    Serial.println("Type 'h' for help, 's' for statistics");
    Serial.println();

    last_update_time = micros();
    last_heartbeat = millis();
}

void loop() {
    unsigned long loop_start = micros();

    // Check for serial commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processCommand();
        } else {
            command_buffer += c;
        }
    }

    // Simulate some work (in real firmware, this would be motor control)
    // We'll add a small delay to simulate processing
    delayMicroseconds(50);  // Simulate 50 μs of work

    // Measure loop execution time
    unsigned long current_time = micros();
    unsigned long loop_time = current_time - loop_start;
    unsigned long elapsed = current_time - last_update_time;

    // Update statistics
    loop_count++;
    total_loop_time_us += loop_time;

    if (loop_time > max_loop_time_us) {
        max_loop_time_us = loop_time;
    }
    if (loop_time < min_loop_time_us) {
        min_loop_time_us = loop_time;
    }

    // Calculate jitter (deviation from target)
    float jitter = abs((long)elapsed - (long)LOOP_PERIOD_US);
    jitter_sum += jitter;
    jitter_samples++;

    // Update histogram
    if (loop_time <= 100) timing_histogram[0]++;
    else if (loop_time <= 200) timing_histogram[1]++;
    else if (loop_time <= 400) timing_histogram[2]++;
    else if (loop_time <= 600) timing_histogram[3]++;
    else if (loop_time <= 800) timing_histogram[4]++;
    else if (loop_time <= 1000) timing_histogram[5]++;
    else if (loop_time <= 1200) timing_histogram[6]++;
    else if (loop_time <= 1500) timing_histogram[7]++;
    else if (loop_time <= 2000) timing_histogram[8]++;
    else timing_histogram[9]++;

    // Check for timing violations
    if (elapsed > LOOP_PERIOD_US) {
        timing_violations++;

        // Print warning for significant violations
        if (elapsed > LOOP_PERIOD_US * 1.5) {
            Serial.print("[WARNING] Timing violation: ");
            Serial.print(elapsed);
            Serial.print(" μs (");
            Serial.print(elapsed - LOOP_PERIOD_US);
            Serial.println(" μs over target)");
        }
    }

    // Heartbeat every 10 seconds
    if (millis() - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = millis();

        float avg_time = (float)total_loop_time_us / loop_count;
        float cpu_usage = avg_time / LOOP_PERIOD_US * 100.0;
        float violation_rate = (float)timing_violations / loop_count * 100.0;

        Serial.print("[HEARTBEAT] Loops: ");
        Serial.print(loop_count);
        Serial.print(" | Avg: ");
        Serial.print(avg_time, 1);
        Serial.print(" μs | CPU: ");
        Serial.print(cpu_usage, 1);
        Serial.print("% | Violations: ");
        Serial.print(violation_rate, 2);
        Serial.println("%");
    }

    // Maintain loop timing
    unsigned long loop_end = micros();
    unsigned long total_elapsed = loop_end - last_update_time;

    if (total_elapsed < LOOP_PERIOD_US) {
        delayMicroseconds(LOOP_PERIOD_US - total_elapsed);
    }

    last_update_time = micros();
}
