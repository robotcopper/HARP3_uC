# Results

The raw output of the measurements behind [the comparison](../README.md), as
the scripts of [`benchmarks/tools`](../../../benchmarks/tools) wrote it. Each
file is named `<label>_<date>_<time>.txt`, and `figures.py` reads them by label.

## Format

`measure.py` writes one section per phase:

* `[no_joint]`: odometry and stats subscribed, no JointState.
* `[joint]`: with JointState at 100 Hz, or at 99 Hz when the label ends with
  `99hz`.
* `[joint_ping]`: the same, with 100 pings.

Each section gives:

* the idle time;
* the CPU share of each task, the XIP cache counters, what `rmw_wait()` did
  and the subscription lookup time, for the micro-ROS firmware;
* the uplink, per odometry message;
* the downlink, per second;
* their sum, per stats period: the figure to compare.

Distributions read `n= min= p10= median= mean= p90= max= sd=`, in
microseconds.

`service_rtt.py` writes:

* `[late_join]`, with five delays in milliseconds;
* `[idle]` and `[joint]`, each with the same lines plus the service round trip.

A `profile of phase joint:` line names the absolute path the profile was
written to on the machine that measured. The profile is the `_profile.txt`
file with the same label next to it: one line per batch of 64-byte flash
buckets, `bucket:samples` in hexadecimal, for `profile_report.py` with the ELF
of that build.

## micro-ROS, 125 MHz

Successive states of the micro-ROS firmware, before it became
`benchmarks/microros_rp2040_basic`. Each run changes one thing from the
previous one. JointState is at 100 Hz up to `cadence_raced`.

| File | Change |
| --- | --- |
| `rmw_O3_cpu_stats_20261107_170938.txt` | rmw_zenoh_pico at `-O3` rather than its forced `-O0`; per-task CPU in the stats |
| `rmw_O3_no_entry_trace_20261107_171554.txt` | function entry trace no longer compiled in |
| `profiler_20261107_172408.txt`, `_profile.txt` | program counter sampler built in; the layout changed, the code that runs did not |
| `hot_ram_20261107_173939.txt`, `_profile.txt` | the 183 hottest functions run from RAM |
| `wait_counts_20261107_175706.txt` | counters in `rmw_wait()`, which show the polling |
| `cadence_20261107_182100.txt` | a cadence task and a guard condition instead of an `rcl` timer; guard condition fixes |
| `cadence_raced_20261107_182902.txt` | `rmw_wait()` no longer sleeps once a condition has triggered |
| `cadence_raced_99hz_20261107_183148.txt` | the same firmware, JointState at 99 Hz |
| `ss_probe_99hz_20261107_183947.txt` | the same firmware, with the host's TCP state sampled by `ss` during the run |
| `rx_split_99hz_20261107_184814.txt` | samples stamped on arrival in rmw_zenoh_pico: `before_rmw_us` and `in_executor_us` |
| `rx_split_99hz_profile_20261107_185040.txt`, `_profile.txt` | the same firmware, profiled |

## micro-ROS, 200 MHz

| File | Firmware |
| --- | --- |
| `clock200_99hz_20261107_194131.txt` | `rx_split` at 200 MHz; subscription lookup timed |
| `clock200_rx_cache_99hz_20261107_194720.txt` | with zenoh-pico's RX cache |
| `microros_rp2040_basic_99hz_20261107_231629.txt` | `benchmarks/microros_rp2040_basic`: transient local, services, query leak fixed |
| `microros_rp2040_basic_service_20261107_234202.txt` | the same, `service_rtt.py` |

## Pico-ROS

| File | Firmware |
| --- | --- |
| `picoros_basic_20261107_175042.txt` | picoros-usb-eth's `examples/rp2040_basic`, 125 MHz, JointState at 100 Hz |
| `picoros_basic_99hz_20261107_193630.txt` | the same, JointState at 99 Hz |
| `picoros_basic_clock200_99hz_20261107_200637.txt` | the same at 200 MHz |
| `picoros_rp2040_basic_99hz_20261107_231111.txt` | `benchmarks/picoros_rp2040_basic`, 200 MHz: the example with `pico/ping` |
| `picoros_rp2040_basic_service_20261107_231239.txt` | the same, `service_rtt.py` |

Pico-ROS times the downlink from the rmw attachment, which the host stamps
0.12 ms after `header.stamp` on average: add that to its sums before comparing
them with micro-ROS's.

## Host only

| File | Run |
| --- | --- |
| `host_ref_peer_20261107_190729.txt` | `host_reference.py`, peer mode |
| `host_ref_client_20261107_190841.txt` | `host_reference.py --mode client`, through the router |
| `host_ref_service_peer_20261107_230738.txt` | `service_rtt.py --host-reference peer` |
| `host_ref_service_client_20261107_230801.txt` | `service_rtt.py --host-reference client` |
