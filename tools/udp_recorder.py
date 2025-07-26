import socket
import struct
import threading
import time
import os
import argparse
from queue import Queue, Empty
import sys
import logging
from collections import deque
import statistics
import math

# struct-Format: <Qffff (little-endian, 1x unsigned long long, 4x float)
# Q: 8 Bytes (Timestamp), f: 4 Bytes (Bx, By, Bz, Temp)
PACKET_STRUCT_FORMAT = "<Qffff"
PACKET_SIZE = struct.calcsize(PACKET_STRUCT_FORMAT)

def new_filename(output_dir, prefix):
    """Generates a new filename with a timestamp."""
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(output_dir, f"{prefix}_{timestamp}.bin")

class AnomalyDetector:
    """Detects anomalies in magnetic field data based on standard deviation."""
    def __init__(self, window_size=100, threshold_std_dev=5.0, cooldown_s=5):
        self.window = deque(maxlen=window_size)
        self.threshold_std_dev = threshold_std_dev
        self.cooldown_s = cooldown_s
        self.last_anomaly_time = 0
        self.mean = 0
        self.std_dev = 0

    def check(self, ts, bx, by, bz) -> bool:
        """Checks a new data point and returns True if it's an anomaly."""
        current_time = time.time()
        if (current_time - self.last_anomaly_time) < self.cooldown_s:
            return False

        magnitude = math.sqrt(bx**2 + by**2 + bz**2)
        
        # Wait until the window is full for stable statistics
        if len(self.window) < self.window.maxlen:
            self.window.append(magnitude)
            return False

        self.mean = statistics.mean(self.window)
        self.std_dev = statistics.stdev(self.window)
        
        # Avoid division by zero if the signal is flat
        if self.std_dev == 0:
            self.window.append(magnitude)
            return False

        deviation = abs(magnitude - self.mean)
        
        if deviation > self.threshold_std_dev * self.std_dev:
            self.last_anomaly_time = current_time
            return True
            
        self.window.append(magnitude)
        return False

class DataWriter:
    """Handles writing data to one or more files with rotation logic."""
    def __init__(self, output_dir, prefix, max_file_size, max_duration, split_xyz=False, drop_boring=False, logger=None):
        self.output_dir = output_dir
        self.prefix = prefix
        os.makedirs(output_dir, exist_ok=True)
        self.max_file_size = max_file_size
        self.max_duration = max_duration
        self.split_xyz = split_xyz
        self.drop_boring = drop_boring
        self.logger = logger or logging.getLogger(__name__)

        self.file = None
        self.file_x = self.file_y = self.file_z = self.file_ts = None
        self.filename = None

        self.start_time = time.time()
        self.file_start_time = self.start_time
        self.bytes_written_current_file = 0
        self.total_bytes_written = 0
        self.file_count = 0
        self.anomaly_in_current_file = False
        self._open_new_file()

    def _finalize_current_file(self):
        """Handles closing and potentially deleting the file(s) currently being written to."""
        if not self.filename:
            return

        should_delete = self.drop_boring and not self.anomaly_in_current_file
        
        # Store paths before closing handles and nullifying state
        paths_to_delete = []
        if should_delete:
            self.logger.info(f"Dropping boring file(s) based on: {os.path.basename(self.filename)}")
            if self.split_xyz:
                # self.filename is the base path like "data/udp_capture_..."
                for suffix in ['_x.bin', '_y.bin', '_z.bin', '_ts.bin']:
                    paths_to_delete.append(f"{self.filename}{suffix}")
            else:
                # self.filename is the full path with .bin extension
                paths_to_delete.append(self.filename)

        # First, close all file handles
        files_to_close = [self.file, self.file_x, self.file_y, self.file_z, self.file_ts]
        for f in files_to_close:
            if f and not f.closed:
                f.close()

        # Now, delete the physical files if needed
        if should_delete:
            for path in paths_to_delete:
                try:
                    os.remove(path)
                except OSError as e:
                    self.logger.error(f"Error deleting boring file {path}: {e}")
        
        # Nullify handles and filename to prevent reuse
        self.file = self.file_x = self.file_y = self.file_z = self.file_ts = None
        self.filename = None
        
    def _open_new_file(self):
        """Closes the current file(s) and opens new ones."""
        self._finalize_current_file()

        self.file_count += 1
        base_filename_with_ext = new_filename(self.output_dir, self.prefix)
        base_path, _ = os.path.splitext(base_filename_with_ext)
        
        self.logger.info(f"Rotating file. New base: {os.path.basename(base_path)}")

        if self.split_xyz:
            self.filename = base_path
            self.file_x = open(f"{base_path}_x.bin", "wb")
            self.file_y = open(f"{base_path}_y.bin", "wb")
            self.file_z = open(f"{base_path}_z.bin", "wb")
            self.file_ts = open(f"{base_path}_ts.bin", "wb")
        else:
            self.filename = base_filename_with_ext
            self.file = open(self.filename, "wb")

        self.file_start_time = time.time()
        self.bytes_written_current_file = 0
        self.anomaly_in_current_file = False # Reset for the new file

    def write(self, data):
        """Writes data to the currently open file(s)."""
        bytes_to_add = 0
        if self.split_xyz:
            num_packets = len(data) // PACKET_SIZE
            for i in range(num_packets):
                offset = i * PACKET_SIZE
                chunk = data[offset : offset + PACKET_SIZE]
                ts, bx, by, bz, _temp = struct.unpack(PACKET_STRUCT_FORMAT, chunk)
                
                self.file_ts.write(struct.pack("<Q", ts))
                self.file_x.write(struct.pack("<f", bx))
                self.file_y.write(struct.pack("<f", by))
                self.file_z.write(struct.pack("<f", bz))
                # 8 (ts) + 4 (bx) + 4 (by) + 4 (bz) = 20 bytes per packet
                bytes_to_add += 20
        else:
            self.file.write(data)
            bytes_to_add = len(data)

        self.bytes_written_current_file += bytes_to_add
        self.total_bytes_written += bytes_to_add
        self.flush()

        # Check for file rotation
        if self.max_file_size and self.bytes_written_current_file >= self.max_file_size:
            self._open_new_file()
        elif self.max_duration and (time.time() - self.file_start_time) >= self.max_duration:
            self._open_new_file()

    def record_file_anomaly(self):
        """Flags that an anomaly has occurred for the current file."""
        if not self.anomaly_in_current_file:
            self.anomaly_in_current_file = True
            self.logger.info(f"Anomaly flagged for current file: {self.get_current_filename()}")

    def flush(self):
        """Flushes buffers to disk."""
        files_to_flush = [self.file, self.file_x, self.file_y, self.file_z, self.file_ts]
        for f in files_to_flush:
            if f and not f.closed:
                f.flush()

    def size_total(self):
        """Returns the total size of all written data in bytes."""
        return self.total_bytes_written

    def duration(self):
        """Returns the total recording duration in seconds."""
        return time.time() - self.start_time

    def close(self):
        """Safely closes all open file handles."""
        self._finalize_current_file()

    def get_current_filename(self):
        """Returns a display name for the current file(s)."""
        if not self.filename:
            return "N/A"
        if self.split_xyz:
            return os.path.basename(self.filename) + "_[x,y,z,ts].bin"
        else:
            return os.path.basename(self.filename)

class Statistics:
    """Manages and calculates recording statistics."""
    def __init__(self):
        self.lock = threading.Lock()
        self.packet_count = 0
        self.samples_count = 0
        self.packets_last_second = 0
        self.samples_last_second = 0
        self.last_value = None
        self.last_two_timestamps = []
        self.anomalies_detected = 0
        self.last_anomaly = None

    def increment(self, packet_count, sample_count):
        with self.lock:
            self.packet_count += packet_count
            self.samples_count += sample_count
            self.packets_last_second += packet_count
            self.samples_last_second += sample_count

    def record_anomaly(self, value):
        with self.lock:
            self.anomalies_detected += 1
            self.last_anomaly = value

    def set_last(self, value):
        with self.lock:
            self.last_value = value
            ts = value[0]
            if len(self.last_two_timestamps) == 2:
                self.last_two_timestamps.pop(0)
            self.last_two_timestamps.append(ts)

    def get_last(self):
        with self.lock:
            return self.last_value, self.last_anomaly

    def get_sampling_rate(self):
        """Estimates the sampling rate from the last two timestamps."""
        with self.lock:
            if len(self.last_two_timestamps) < 2:
                return 0.0
            # Assumption: timestamps are in milliseconds from the sensor
            delta_ms = self.last_two_timestamps[1] - self.last_two_timestamps[0]
            if delta_ms <= 0:
                return 0.0
            return 1000.0 / delta_ms

    def reset_pps_sps(self):
        with self.lock:
            pps = self.packets_last_second
            sps = self.samples_last_second
            self.packets_last_second = 0
            self.samples_last_second = 0
            return pps, sps

def print_statistics(stats, writer, port, stop_event):
    """Periodically prints statistics to the console."""
    while not stop_event.is_set():
        time.sleep(1)
        pps, sps = stats.reset_pps_sps()
        sampling_rate = stats.get_sampling_rate()
        total_size_mb = writer.size_total() / (1024 * 1024)
        duration_sec = writer.duration()
        last_val, last_anomaly = stats.get_last()

        # ANSI escape codes to clear the screen and position the cursor
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(f"UDP Data Recorder | Listening on port {port}\n")
        sys.stdout.write("="*80 + "\n")
        sys.stdout.write(
            f"📈 Status: {pps:5d} pps | Rate: {sampling_rate:7.2f} Hz | 📁 File: {writer.get_current_filename()}\n"
            f"💾 Total Size: {total_size_mb:8.2f} MB | ⏱ Duration: {duration_sec:8.1f} s | ⚠ Anomalies: {stats.anomalies_detected}\n"
        )
        sys.stdout.write("-"*80 + "\n")
        
        sys.stdout.write("Latest BMM350 Value:\n")
        sys.stdout.write("  Timestamp            | B_x [uT]  | B_y [uT]  | B_z [uT]  | Temp [C]\n")
        sys.stdout.write("---------------------+-----------+-----------+-----------+----------\n")
        if last_val:
            sys.stdout.write(f"  {last_val[0]:<18d} | {last_val[1]:9.3f} | {last_val[2]:9.3f} | {last_val[3]:9.3f} | {last_val[4]:8.2f}\n")
        else:
            sys.stdout.write("  Waiting for data...\n")

        sys.stdout.write("\nLast Detected Anomaly:\n")
        if last_anomaly:
            sys.stdout.write(f"  {last_anomaly[0]:<18d} | {last_anomaly[1]:9.3f} | {last_anomaly[2]:9.3f} | {last_anomaly[3]:9.3f} | {last_anomaly[4]:8.2f}\n")
        else:
            sys.stdout.write("  None detected yet.\n")

        sys.stdout.flush()

def udp_receiver(sock, queue, stats, detector, writer, stop_event, logger):
    """Receives UDP packets and puts them into a queue."""
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(65535)
            logger.debug(f"Received {len(data)} bytes from {addr}")
            
            if len(data) % PACKET_SIZE != 0:
                logger.warning(f"Discarding packet from {addr}: size {len(data)} not multiple of {PACKET_SIZE}")
                continue
                
            queue.put(data)
            
            num_packets = len(data) // PACKET_SIZE
            stats.increment(num_packets, num_packets)

            if num_packets > 0:
                # Use the last packet in the UDP datagram for stats and anomaly detection
                offset = (num_packets - 1) * PACKET_SIZE
                last_packet_data = data[offset : offset + PACKET_SIZE]
                ts, bx, by, bz, temp = struct.unpack(PACKET_STRUCT_FORMAT, last_packet_data)
                
                last_value = (ts, bx, by, bz, temp)
                stats.set_last(last_value)

                if detector.check(ts, bx, by, bz):
                    stats.record_anomaly(last_value)
                    writer.record_file_anomaly()
                    logger.warning(f"Anomaly detected! Value: {last_value}")

        except socket.error:
            logger.info("Socket error, probably closed by main thread.")
            break
        except Exception as e:
            logger.error(f"Error in udp_receiver: {e}", exc_info=True)
            break

def writer_thread(writer, queue, stop_event, stop_after_s, stop_after_bytes):
    """Gets data from the queue and writes it to disk."""
    start_time = time.time()
    while not stop_event.is_set():
        try:
            data = queue.get(timeout=1)
            writer.write(data)
            queue.task_done()
        except Empty:
            pass # Timeout is ok, allows checking stop conditions

        if stop_after_s and (time.time() - start_time) >= stop_after_s:
            print(f"\nStop condition met: Recorded for {stop_after_s} seconds.")
            stop_event.set()
        
        if stop_after_bytes and writer.size_total() >= stop_after_bytes:
            total_mb = writer.size_total() / (1024 * 1024)
            print(f"\nStop condition met: Wrote {total_mb:.2f} MB.")
            stop_event.set()

def print_summary(stats, writer):
    """Prints a final summary of the session."""
    duration = writer.duration()
    total_mb = writer.size_total() / (1024 * 1024)
    avg_rate = (stats.samples_count / duration) if duration > 0 else 0

    print("\n" + "="*80)
    print(" S U M M A R Y ")
    print("="*80)
    print(f"  Total recording duration: {duration:.2f} seconds")
    print(f"  Total data written:     {total_mb:.3f} MB")
    print(f"  Total packets received:   {stats.packet_count}")
    print(f"  Total samples captured:   {stats.samples_count}")
    print(f"  Average sample rate:    {avg_rate:.2f} Hz")
    print(f"  Number of files created:  {writer.file_count}")
    print(f"  Anomalies detected:       {stats.anomalies_detected}")
    print("="*80)

def main():
    parser = argparse.ArgumentParser(
        description="UDP data recorder with file rotation, statistics, and anomaly detection.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    # Network and File Arguments
    parser.add_argument("--port", type=int, default=12345, help="UDP port to listen on.")
    parser.add_argument("--output-dir", type=str, default="data", help="Directory to save data files.")
    parser.add_argument("--prefix", type=str, default="udp_capture", help="Prefix for output filenames.")
    parser.add_argument("--split-xyz", action="store_true", help="Split data into separate Bx, By, Bz, and Ts files.")

    # Rotation Arguments
    parser.add_argument("--max-file-size", type=int, default=100, help="Max file size in MB before rotating (0 for unlimited).")
    parser.add_argument("--max-duration", type=int, default=600, help="Max file duration in seconds before rotating (0 for unlimited).")

    # Stop Arguments
    parser.add_argument("--stop-after-seconds", type=int, default=0, help="Stop recording after this many seconds (0 for unlimited).")
    parser.add_argument("--stop-after-mb", type=int, default=0, help="Stop recording after this total MB are written (0 for unlimited).")

    # Anomaly Detection Arguments
    parser.add_argument("--anomaly-window", type=int, default=100, help="Window size for anomaly detection moving average.")
    parser.add_argument("--anomaly-threshold", type=float, default=5.0, help="Standard deviation multiplier for anomaly threshold.")
    parser.add_argument("--anomaly-cooldown", type=int, default=5, help="Cooldown in seconds after detecting an anomaly.")
    parser.add_argument("--drop-boring", action="store_true", help="Delete files that contain no detected anomalies upon rotation or exit.")
    
    # Debugging
    parser.add_argument("--debug", type=str, metavar="LOG_FILE", help="Enable debug logging to the specified file.")
    args = parser.parse_args()

    # Configure logger
    logger = logging.getLogger("udp_recorder")
    if args.debug:
        handler = logging.FileHandler(args.debug, mode='w')
        handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    else:
        logger.addHandler(logging.NullHandler())

    # Set up socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening for UDP packets on port {args.port}...")

    # Instantiate helper classes
    stats = Statistics()
    detector = AnomalyDetector(
        window_size=args.anomaly_window,
        threshold_std_dev=args.anomaly_threshold,
        cooldown_s=args.anomaly_cooldown
    )
    writer = DataWriter(
        args.output_dir,
        args.prefix,
        args.max_file_size * 1024 * 1024 if args.max_file_size > 0 else None,
        args.max_duration if args.max_duration > 0 else None,
        split_xyz=args.split_xyz,
        drop_boring=args.drop_boring,
        logger=logger
    )
    data_queue = Queue(maxsize=1000)
    stop_event = threading.Event()

    # Create threads
    threads = [
        threading.Thread(target=print_statistics, args=(stats, writer, args.port, stop_event), daemon=True),
        threading.Thread(target=udp_receiver, args=(sock, data_queue, stats, detector, writer, stop_event, logger), daemon=True),
        threading.Thread(
            target=writer_thread,
            args=(writer, data_queue, stop_event, args.stop_after_seconds, args.stop_after_mb * 1024 * 1024 if args.stop_after_mb > 0 else 0),
            daemon=True,
        ),
    ]

    for t in threads:
        t.start()

    try:
        # Main thread waits until a stop signal is set
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down...")
    finally:
        if not stop_event.is_set():
            stop_event.set()
            
        print("Shutdown signal sent. Waiting for threads to terminate...")
        
        # Stop receiver thread first to prevent queue pile-up
        threads[1].join(timeout=2.0)
        # Stop writer thread
        threads[2].join(timeout=5.0) # Allow more time for final write
        
        writer.close()
        sock.close()
        
        print_summary(stats, writer)
        print("Recording finished. ✅")

if __name__ == "__main__":
    main()
