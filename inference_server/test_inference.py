#!/usr/bin/env python3
"""Test script for inference server"""
import requests
import json
import time

SERVER_URL = "http://localhost:8080"

def test_health():
    """Test health endpoint"""
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{SERVER_URL}/health", timeout=5)
        print(f"Status: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")
        return response.status_code == 200
    except Exception as e:
        print(f"Error: {e}")
        return False

def test_generate(command, **kwargs):
    """Test generate_bt endpoint"""
    print(f"\n{'='*80}")
    print(f"Testing generation with command: '{command}'")
    print(f"Parameters: {kwargs}")
    print(f"{'='*80}")
    
    payload = {
        "command": command,
        **kwargs
    }
    
    try:
        start = time.time()
        response = requests.post(
            f"{SERVER_URL}/generate_bt",
            json=payload,
            timeout=120
        )
        elapsed = time.time() - start
        
        print(f"\nStatus: {response.status_code}")
        result = response.json()
        print(f"Success: {result.get('success')}")
        print(f"Method: {result.get('method_used')}")
        print(f"Generation time: {result.get('generation_time_ms')}ms")
        print(f"Total time: {elapsed*1000:.0f}ms")
        
        if result.get('success'):
            print(f"\nGenerated XML:")
            print(result.get('bt_xml'))
        else:
            print(f"\nError: {result.get('error')}")
        
        return result.get('success', False)
        
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    print("Starting inference server tests...")
    print(f"Server: {SERVER_URL}\n")
    
    # Test health
    if not test_health():
        print("\n❌ Health check failed. Is the server running?")
        return
    
    print("\n✅ Health check passed\n")
    time.sleep(1)
    
    # Test cases
    test_cases = [
        {
            "command": "rotate left",
            "use_few_shot": True,
            "prompt_format": "alpaca",
            "use_query_rewriting": True,
            "temperature": 0.3,
            "max_tokens": 512
        },
        {
            "command": "move forward 2 meters",
            "use_few_shot": True,
            "prompt_format": "alpaca",
            "use_query_rewriting": True,
            "temperature": 0.3,
            "max_tokens": 512
        },
        {
            "command": "pick up the red cup",
            "use_few_shot": True,
            "prompt_format": "alpaca",
            "use_query_rewriting": True,
            "temperature": 0.3,
            "max_tokens": 512
        }
    ]
    
    results = []
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n{'#'*80}")
        print(f"TEST CASE {i}/{len(test_cases)}")
        print(f"{'#'*80}")
        success = test_generate(**test_case)
        results.append((test_case['command'], success))
        time.sleep(2)  # Small delay between tests
    
    # Summary
    print(f"\n{'='*80}")
    print("SUMMARY")
    print(f"{'='*80}")
    for command, success in results:
        status = "✅" if success else "❌"
        print(f"{status} {command}")
    
    total = len(results)
    passed = sum(1 for _, s in results if s)
    print(f"\nTotal: {passed}/{total} passed")

if __name__ == "__main__":
    main()
