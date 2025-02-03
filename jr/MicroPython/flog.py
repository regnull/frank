import time
import sys

class Log:
    def __init__(self, basename: str):
        for i in range(1000):
            filename = f"{basename}_{i:03d}.log"
            try:
                # Try to open the file in read mode - if it succeeds, the file exists
                # so we continue to the next iteration
                with open(filename, "r") as f:
                    continue
            except OSError:
                # If we get an OSError, the file doesn't exist
                # so we've found our filename - break the loop
                self.filename = filename
                break
        else:
            # If we exhausted the loop without finding a free filename
            raise OSError("No available log file names")
        
        self.file = open(self.filename, "w")
        self.start_time = time.ticks_ms()
        self.log_count = 0

    def print(self, message: str):
        self.log_count += 1
        timestamp = time.ticks_ms() - self.start_time
        self.file.write(f"{timestamp} {message}\n")
        if self.log_count % 5 == 0:
            self.file.flush()
        
    def print_exception(self, e: Exception):
        self.print(f"Error: {e}")
        # self.file.write(f"{e}\n")
        sys.print_exception(e, self.file)
        self.file.flush()

    def close(self):
        self.file.close()
