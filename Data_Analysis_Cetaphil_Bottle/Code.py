import re
from datetime import datetime
import matplotlib.pyplot as plt

# -----------------------------
# CONFIG
# -----------------------------
LOG_PATH = "serial_log.txt"   # <-- change this to your filename

FSR_KEYS = ["PB0","PA7","PA6","PA5","PA4","PA3","PA2","PA1","PA0"]
CUR_KEYS = ["S0","S1","S2","S3","S4"]

# Accept 1 or 2 digit hour:
# 4:33:44:455 -> ...
# 14:33:44:455 -> ...
TIME_PREFIX_RE = re.compile(
    r"^(?P<h>\d{1,2}):(?P<m>\d{2}):(?P<s>\d{2}):(?P<ms>\d{3})\s*->\s*(?P<rest>.*)$"
)

FSR_RE = re.compile(r"^FSR Live:\s*(?P<body>.+)$")

CUR_RE = re.compile(
    r"^(?P<millis>\d+)\s*,\s*(?P<pulse>\d+)\s*,\s*"
    r"S0=(?P<S0>[-+]?\d*\.?\d+)\s*mA,\s*"
    r"S1=(?P<S1>[-+]?\d*\.?\d+)\s*mA,\s*"
    r"S2=(?P<S2>[-+]?\d*\.?\d+)\s*mA,\s*"
    r"S3=(?P<S3>[-+]?\d*\.?\d+)\s*mA,\s*"
    r"S4=(?P<S4>[-+]?\d*\.?\d+)\s*mA\s*$"
)

FSR_KV_RE = re.compile(r"(?P<k>PB0|PA7|PA6|PA5|PA4|PA3|PA2|PA1|PA0)=(?P<v>[-+]?\d*\.?\d+)")

def parse_time_prefix(line: str):
    m = TIME_PREFIX_RE.match(line.strip())
    if not m:
        return None, line.strip(), False

    h = int(m.group("h"))
    mi = int(m.group("m"))
    s = int(m.group("s"))
    ms = int(m.group("ms"))
    rest = m.group("rest").strip()

    dt = datetime(2000, 1, 1, h, mi, s, ms * 1000)
    return dt, rest, True

def main():
    fsr_rows = []
    cur_rows = []

    first_host_dt = None
    first_millis = None

    with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            # Skip noise
            if line.startswith("===") or line.startswith("----") or "Closed the serial port" in line:
                continue
            if "Invalid command" in line or "Enter next command" in line:
                continue
            if "STABLE" in line or "FSR Snapshot" in line or line.startswith("====================================="):
                continue
            if "Serial Commands Reference" in line or "Servos" in line:
                continue
            if "Moving servos" in line or "Received:" in line:
                continue

            host_dt, rest, has_host_time = parse_time_prefix(line)
            if has_host_time and first_host_dt is None:
                first_host_dt = host_dt

            # -------- FSR --------
            mf = FSR_RE.match(rest)
            if mf:
                body = mf.group("body")
                pairs = dict((m.group("k"), float(m.group("v"))) for m in FSR_KV_RE.finditer(body))
                if not pairs:
                    continue

                row = {"host_dt": host_dt, "t": None}
                for k in FSR_KEYS:
                    row[k] = pairs.get(k, None)
                fsr_rows.append(row)
                continue

            # -------- Current --------
            mc = CUR_RE.match(rest)
            if mc:
                millis_val = int(mc.group("millis"))
                pulse = int(mc.group("pulse"))

                if first_millis is None:
                    first_millis = millis_val

                row = {"host_dt": host_dt, "millis": millis_val, "pulse": pulse, "t": None}
                for k in CUR_KEYS:
                    row[k] = float(mc.group(k))
                cur_rows.append(row)
                continue

    def compute_t(row):
        if first_host_dt is not None and row.get("host_dt") is not None:
            return (row["host_dt"] - first_host_dt).total_seconds()
        if "millis" in row and first_millis is not None:
            return (row["millis"] - first_millis) / 1000.0
        return None

    for r in fsr_rows:
        r["t"] = compute_t(r)
    for r in cur_rows:
        r["t"] = compute_t(r)

    fsr_rows = [r for r in fsr_rows if r["t"] is not None]
    cur_rows = [r for r in cur_rows if r["t"] is not None]

    if not fsr_rows and not cur_rows:
        print("No data found. Check LOG_PATH and log format.")
        return

    # Plot FSR
    if fsr_rows:
        t_fsr = [r["t"] for r in fsr_rows]
        plt.figure()
        for k in FSR_KEYS:
            y = [r[k] for r in fsr_rows]
            plt.plot(t_fsr, y, label=k)
        plt.xlabel("Time (s)")
        plt.ylabel("FSR reading")
        plt.title("FSR Live (9 sensors)")
        plt.legend(loc="best")
        plt.grid(True)

    # Plot currents + pulse
    if cur_rows:
        t_cur = [r["t"] for r in cur_rows]
        fig = plt.figure()
        ax1 = fig.add_subplot(111)

        for k in CUR_KEYS:
            y = [r[k] for r in cur_rows]
            ax1.plot(t_cur, y, label=k)

        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Current (mA)")
        ax1.set_title("INA226 Currents + Servo Pulse Width")
        ax1.grid(True)

        ax2 = ax1.twinx()
        pulse_y = [r["pulse"] for r in cur_rows]
        ax2.plot(t_cur, pulse_y, linestyle="--", label="Pulse (us)")
        ax2.set_ylabel("Servo pulse width (us)")

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="best")

    plt.show()

if __name__ == "__main__":
    main()

