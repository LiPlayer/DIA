#!/usr/bin/env python3

import subprocess

def get_topics():
    result = subprocess.run(["ros2", "topic", "list"],
                            capture_output=True, text=True)
    topics = result.stdout.strip().split("\n")
    return [t.strip() for t in topics if t.strip()]


def get_topic_verbose(topic):
    return subprocess.run(
        ["ros2", "topic", "info", topic, "-v"],
        capture_output=True, text=True
    ).stdout


def extract_endpoints(verbose_text):
    """
    返回 (endpoint_type, node_name, reliability)
    endpoint_type ∈ {"PUBLISHER", "SUBSCRIPTION"}
    """
    lines = verbose_text.splitlines()
    endpoints = []

    current_node = None
    in_endpoint = False
    current_type = None

    for line in lines:
        line = line.strip()

        if line.startswith("Node name:"):
            current_node = line.split(":", 1)[1].strip()

        if line.startswith("Endpoint type:"):
            current_type = line.split(":", 1)[1].strip()
            in_endpoint = True
            continue

        if in_endpoint and line.startswith("Reliability:"):
            reliability = line.split(":", 1)[1].strip()
            endpoints.append((current_type, current_node, reliability))
            in_endpoint = False

    return endpoints


def reliability_icon(reliability):
    """
    返回带图标的字符串：
      🟢 BestEffort
      🔵 Reliable
      🟡 Other（例如 Unknown）
    """
    r = reliability.lower()
    if "best" in r:
        return f"🟢 {reliability}"
    elif "reliable" in r:
        return f"🔵 {reliability}"
    else:
        return f"🟡 {reliability}"


def main():
    topics = get_topics()

    print("\n📡 QoS Overview (Publishers 🟣 / Subscribers 🔸 with Reliability icons):\n")

    for topic in topics:
        print(f"\n🔍 Topic: {topic}")

        verbose = get_topic_verbose(topic)
        endpoints = extract_endpoints(verbose)

        if not endpoints:
            print("   ⚠️  No publishers or subscribers.")
            continue

        # Publishers
        for etype, node_name, reliability in endpoints:
            if etype == "PUBLISHER":
                print(f"   [PUB] {node_name}: {reliability_icon(reliability)}")

        # Subscribers
        for etype, node_name, reliability in endpoints:
            if etype == "SUBSCRIPTION":
                print(f"   [SUB] {node_name}: {reliability_icon(reliability)}")


if __name__ == "__main__":
    main()

