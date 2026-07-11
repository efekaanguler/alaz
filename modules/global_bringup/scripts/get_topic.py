#!/usr/bin/env python3
import sys
import yaml
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 get_topic.py <category>.<topic_name>")
        print("Example: python3 get_topic.py camera.image_raw")
        sys.exit(1)

    key_path = sys.argv[1].split('.')
    
    # Path to topics.yaml relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(script_dir, "..", "config", "topics.yaml")
    
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading {yaml_path}: {e}", file=sys.stderr)
        sys.exit(1)

    # Traverse the YAML
    current = data.get('topics', {})
    for key in key_path:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            print(f"Error: Key '{sys.argv[1]}' not found in topics.yaml", file=sys.stderr)
            sys.exit(1)

    # Print the resolved topic string
    print(current)

if __name__ == "__main__":
    main()
