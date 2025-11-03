#!/bin/bash

echo "===================================="
echo "::::::AIRPORT OPERATION TESTS:::::::"
echo "===================================="

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color
OS_NAME=$(uname -s)

TOTAL_TESTS=0
PASSED_TESTS=0

# Compile
compile_ok=true


rm -f logs/air_control_c.log logs/air_control_py.log

# ========== AIR CONTROL ==========
if [ -f "../air_control_c/src/functions.c" ]; then
    echo "Compiling air_control with functions.c..."
    gcc -I../air_control_c/include ../air_control_c/src/main.c ../air_control_c/src/functions.c -o air_c || compile_ok=false
else
    echo "Compiling air_control (functions.c missing)..."
    if [ -d "../air_control_c/include" ]; then
        gcc -I../air_control_c/include ../air_control_c/src/main.c -o air_c || compile_ok=false
    else
        gcc ../air_control_c/src/main.c -o air_c || compile_ok=false
    fi
fi

ls -l /dev/shm | grep shm_pids_
stat /dev/shm/shm_pids_
# ========== GROUND CONTROL ==========
if [ -f "../ground_control/src/functions.c" ]; then
    echo "Compiling ground_control with functions.c..."
    gcc -I../ground_control/include ../ground_control/src/main.c ../ground_control/src/functions.c -o ground || compile_ok=false
else
    echo "Compiling ground_control (functions.c missing)..."
    if [ -d "../ground_control/include" ]; then
        gcc -I../ground_control/include ../ground_control/src/main.c -o ground || compile_ok=false
    else
        gcc ../ground_control/src/main.c -o ground || compile_ok=false
    fi
fi

# ========== RESULT CHECK ==========
if [ "$compile_ok" = false ]; then
    echo "❌ Compilation failed. Skipping tests."
    TOTAL_TESTS=0
    PASSED_TESTS=0
    PERCENTAGE=0
    echo ""
    echo "FINAL RESULT:"
    echo "===================================="
    echo -e "${RED} Tests passed: $PASSED_TESTS/$TOTAL_TESTS"
    echo -e "${RED} Percentage: $PERCENTAGE%"
    echo "❌ DEFICIENT - Build failed"
    exit 2
fi

ipcs -m | awk '/16/ {print $2}' | xargs -I {} ipcrm -m {} 2>/dev/null

run_airport_tests() {
    local implementation=$1  # 'c' o 'py'

    echo "========== Running AIR CONTROL ($implementation) =========="
    if [ "$implementation" = "py" ]; then
        PYTHONUNBUFFERED=1 python3 ../air_control_py/main.py >> logs/air_control_py.log 2>&1 &
        sleep 3
        AIR_PID=$!
    else
        ./air_c > logs/air_control_c.log 2>&1 &
        AIR_PID=$!
    fi

    echo "Running ground_control..."
    ./ground 2>&1 &
    GROUND_PID=$!

    # Test 1: 3Threads was created
    echo -e "${YELLOW}1. VERIFY THREAD CREATION${NC}"
    THREAD_TEST_PASSED=false

    case "$OS_NAME" in
        Darwin)
            echo -e "   OS: $OS_NAME..."
            if nm air_c | grep -q "pthread_create"; then
                echo "   ✅ PASSED - air links against pthread_create $OS_NAME"
                PASSED_TESTS=$((PASSED_TESTS + 1))
                THREAD_TEST_PASSED=true
            else
                echo "   ❌ FAILED - air binary missing pthread_create symbol $OS_NAME"
            fi
            ;;
        Linux)
            echo -e "   OS: $OS_NAME..."
            THREADS_CREATED=""
            WORKER_THREADS=0
            if [ -n "$AIR_PID" ] && ps -p "$AIR_PID" > /dev/null 2>&1; then
                if [ -d "/proc/$AIR_PID/task" ]; then
                    THREADS_CREATED=$(ls -1 "/proc/$AIR_PID/task" 2>/dev/null | wc -l | awk '{print $1}')
                elif ps -o nlwp= -p "$AIR_PID" > /dev/null 2>&1; then
                    THREADS_CREATED=$(ps -o nlwp= -p "$AIR_PID" 2>/dev/null | tr -d ' ')
                fi
            fi

            if [[ "$THREADS_CREATED" =~ ^[0-9]+$ ]]; then
                WORKER_THREADS=$((THREADS_CREATED - 1))
            fi

            if [ "$WORKER_THREADS" -eq 5 ]; then
                echo "   ✅ PASSED - buyer spawned 5 buyer threads $OS_NAME"
                PASSED_TESTS=$((PASSED_TESTS + 1))
                THREAD_TEST_PASSED=true
            else
                echo "   ❌ FAILED - expected 5 worker threads, found $WORKER_THREADS"
            fi
            ;;
        *)
            if nm air_c | grep -q "pthread_create"; then
                echo "   ✅ PASSED2222 - pthread_create symbol present $OS_NAME"
                PASSED_TESTS=$((PASSED_TESTS + 1))
                THREAD_TEST_PASSED=true
            else
                echo "   ❌ FAILED - could not verify thread creation on $OS_NAME"
            fi
            ;;
    esac
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    # Test 2: If requests 3, should buy 3
    echo -e "${YELLOW}2. Mutex validation${NC}"
    if nm air_c | grep -q "pthread_mutex_lock"; then
        echo "   ✅ PASSED - buyer create mutex"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo "   ❌ FAILED - buyer missing pthread create/mutex"
    fi
    TOTAL_TESTS=$((TOTAL_TESTS + 1))


    wait $AIR_PID 2>/dev/null
    wait $GROUND_PID 2>/dev/null

    # Test 3: Validate sold out...
    LAST_LINES=$(tail -7 ./logs/air_control_c.log)
    echo -e "${YELLOW}3. Validate report result${NC}"
    # printf '   <<%q>>\n' "$LAST_LINES"
    if echo "$LAST_LINES" | grep -A4 ":::: End of operations ::::" | grep -q "Takeoffs: 20 Planes: 20"; then
        echo "   ✅ PASSED - Correct report results"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo "   ❌ FAILED - No match in last output"
        echo "   $LAST_LINES"
    fi
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    # Test 4: Verify shared memory was released
    echo -e "${YELLOW}4. Shared memory released${NC}"
    if ipcs -m | grep -q "0x00000010"; then
        echo "   ❌ FAILED - Shared memory segment still present"
    else
        echo "   ✅ PASSED - Shared memory segment released"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    fi
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    wait $AIR_PID
    wait $GROUND_PID

    echo "Checking shared memory..."
    if ipcs -m | grep -q "0x00000010"; then
        echo "   ❌ Shared memory still present"
    else
        echo "   ✅ Shared memory released"
    fi
}






echo "===================================="
echo "::::::::: Running C tests ::::::::::"
echo "===================================="
run_airport_tests "c"


echo "===================================="
echo "::::::::: Running py tests :::::::::"
echo "===================================="

# ========== AIR PY CONTROL ==========
run_airport_tests "py"

wait $AIR_PY_PID 2>/dev/null
wait $GROUND_PID 2>/dev/null

# Calculate percentage
if [ $TOTAL_TESTS -gt 0 ]; then
    PERCENTAGE=$((PASSED_TESTS * 100 / TOTAL_TESTS))
else
    PERCENTAGE=0
fi
# kill $AIR_PID 2>/dev/null
# kill $GROUND_PID 2>/dev/null
echo "===================================="
echo "FINAL RESULT:"
echo "===================================="
# Clean up
# rm -f buyer logs/output.log logs/ticket_office.log ticket_office

# Final evaluation
if [ $PERCENTAGE -ge 98 ]; then
    echo -e "${GREEN} Tests passed: $PASSED_TESTS/$TOTAL_TESTS"
    echo -e "${GREEN} Percentage: $PERCENTAGE%"
    echo "EXCELLENT! Business rules work correctly"
    exit 0
elif [ $PERCENTAGE -ge 80 ]; then
    echo -e "${YELLOW} Tests passed: $PASSED_TESTS/$TOTAL_TESTS"
    echo -e "${YELLOW} Percentage: $PERCENTAGE%"
    echo "⚠️  REGULAR - Some rules need adjustments"
    exit 1
else
    echo -e "${RED} Tests passed: $PASSED_TESTS/$TOTAL_TESTS"
    echo -e "${RED} Percentage: $PERCENTAGE%"
    echo "❌ DEFICIENT - Most rules failed"
    exit 2
fi
