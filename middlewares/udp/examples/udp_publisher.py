#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
UDP JSON Publisher for Testing

Sends JSON messages via UDP for testing the Axon UDP plugin.

Usage:
    python3 udp_publisher.py --port 4242 --rate 10 --count 100

Examples:
    # Send GPS data at 10 Hz
    python3 udp_publisher.py --port 4242 --type gps --rate 10

    # Send CAN data at 100 Hz
    python3 udp_publisher.py --port 4243 --type can --rate 100

    # Send custom JSON
    python3 udp_publisher.py --port 4244 --json '{"timestamp": 123, "value": 42}'
"""

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class MessageGenerator:
    """Generates test messages with timestamps."""

    msg_type: str
    counter: int = 0

    def generate(self) -> dict:
        """Generate a test message based on type."""
        timestamp_ns = int(time.time() * 1e9)

        if self.msg_type == "gps":
            msg = {
                "timestamp": timestamp_ns,
                "latitude": 37.7749 + (self.counter * 0.0001),
                "longitude": -122.4194 + (self.counter * 0.0001),
                "altitude": 10.0,
                "hdop": 1.2,
                "satellites": 12
            }
        elif self.msg_type == "can":
            msg = {
                "timestamp": timestamp_ns,
                "can_id": 0x123 + (self.counter % 10),
                "dlc": 8,
                "data": [self.counter % 256] * 8
            }
        elif self.msg_type == "imu":
            msg = {
                "timestamp": timestamp_ns,
                "acceleration": {
                    "x": 0.1,
                    "y": 0.2,
                    "z": 9.8
                },
                "gyroscope": {
                    "x": 0.01,
                    "y": 0.02,
                    "z": 0.03
                }
            }
        else:
            # Default simple message
            msg = {
                "timestamp": timestamp_ns,
                "counter": self.counter,
                "message": f"test_{self.counter}"
            }

        self.counter += 1
        return msg


def send_udp_messages(
    host: str,
    port: int,
    msg_type: str,
    rate: float,
    count: Optional[int],
    custom_json: Optional[str],
    verbose: bool
) -> None:
    """Send UDP messages at specified rate."""

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    generator = MessageGenerator(msg_type)
    interval = 1.0 / rate if rate > 0 else 0

    sent_count = 0
    total_bytes = 0

    try:
        while count is None or sent_count < count:
            # Generate message
            if custom_json:
                msg = json.loads(custom_json)
                msg["timestamp"] = int(time.time() * 1e9)
            else:
                msg = generator.generate()

            # Serialize to JSON
            msg_bytes = json.dumps(msg, separators=(',', ':')).encode('utf-8')

            # Send via UDP
            sock.sendto(msg_bytes, (host, port))

            sent_count += 1
            total_bytes += len(msg_bytes)

            if verbose:
                print(f"[{sent_count}] Sent {len(msg_bytes)} bytes to {host}:{port}")
                if sent_count % 10 == 0:
                    print(f"    {json.dumps(msg)}")

            # Rate limiting
            if interval > 0:
                time.sleep(interval)

    except KeyboardInterrupt:
        print(f"\nInterrupted after sending {sent_count} messages")
    finally:
        sock.close()
        print(f"\nSummary:")
        print(f"  Messages sent: {sent_count}")
        print(f"  Total bytes:   {total_bytes}")
        print(f"  Average size:  {total_bytes / sent_count if sent_count > 0 else 0:.1f} bytes")


def main():
    parser = argparse.ArgumentParser(
        description="UDP JSON Publisher for Axon Testing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        "--host", "-H",
        default="127.0.0.1",
        help="Target host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", "-p",
        type=int,
        required=True,
        help="Target UDP port"
    )
    parser.add_argument(
        "--type", "-t",
        choices=["gps", "can", "imu", "simple"],
        default="simple",
        help="Message type to generate (default: simple)"
    )
    parser.add_argument(
        "--rate", "-r",
        type=float,
        default=10.0,
        help="Messages per second (default: 10)"
    )
    parser.add_argument(
        "--count", "-c",
        type=int,
        default=None,
        help="Number of messages to send (default: infinite)"
    )
    parser.add_argument(
        "--json", "-j",
        type=str,
        default=None,
        help="Custom JSON message to send (timestamp will be added)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print each message"
    )

    args = parser.parse_args()

    print(f"UDP JSON Publisher")
    print(f"  Target: {args.host}:{args.port}")
    print(f"  Type:   {args.type}")
    print(f"  Rate:   {args.rate} Hz")
    print(f"  Count:  {args.count if args.count else 'infinite'}")
    print()

    send_udp_messages(
        args.host,
        args.port,
        args.type,
        args.rate,
        args.count,
        args.json,
        args.verbose
    )


if __name__ == "__main__":
    main()
