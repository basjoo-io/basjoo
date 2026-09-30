#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 2 ]; then
    echo "Usage: $0 <file_to_check> <test_pattern>"
    echo "Example: $0 '.git' 'src/**/*.test.ts'"
    exit 1
fi

POLLUTION_CHECK="$1"
TEST_PATTERN="$2"

echo "Searching for test that creates: $POLLUTION_CHECK"
echo "Test pattern: $TEST_PATTERN"
echo ""

if [ -e "$POLLUTION_CHECK" ]; then
    echo "ERROR: $POLLUTION_CHECK already exists before any test runs."
    echo "Remove it first, then re-run this script."
    exit 1
fi

TEST_FILES=$(find . -name '*.test.ts' -path "*${TEST_PATTERN}*" 2>/dev/null | sort)
TOTAL=$(echo "$TEST_FILES" | grep -c . || true)

if [ "$TOTAL" -eq 0 ]; then
    echo "ERROR: No test files matched pattern: $TEST_PATTERN"
    exit 1
fi

echo "Found $TOTAL test files"
echo ""

COUNT=0
while IFS= read -r TEST_FILE; do
    COUNT=$((COUNT + 1))

    if [ -e "$POLLUTION_CHECK" ]; then
        echo "WARNING: Pollution appeared after previous test — cleaning up"
        rm -rf "$POLLUTION_CHECK"
    fi

    echo "[$COUNT/$TOTAL] Testing: $TEST_FILE"

    test_exit=0
    npm test "$TEST_FILE" >/dev/null 2>&1 || test_exit=$?

    if [ -e "$POLLUTION_CHECK" ]; then
        echo ""
        echo "FOUND POLLUTER!"
        echo "   Test: $TEST_FILE"
        echo "   Created: $POLLUTION_CHECK"
        if [ "$test_exit" -ne 0 ]; then
            echo "   Note: test also failed (exit $test_exit)"
        fi
        echo ""
        echo "Pollution details:"
        ls -la "$POLLUTION_CHECK"
        echo ""
        echo "To investigate:"
        echo "  npm test $TEST_FILE    # Run just this test"
        exit 1
    fi
done <<<"$TEST_FILES"

echo ""
echo "No polluter found - all tests clean!"
exit 0
