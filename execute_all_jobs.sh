#!/bin/bash

# Create the "completed" folder if it doesn't exist
mkdir -p completed

# Function to handle errors
function handle_errors {
    # Ignore the error and continue
    return 0
}

# Main loop
while true; do
    # Find all the .sh files in the "jobs" folder and sort them
    scripts=$(find jobs -type f -name "*.sh" | sort)

    # If there are no scripts left, exit the loop
    if [ -z "$scripts" ]; then
        break
    fi

   

    # Iterate through the scripts
    for script in $scripts; do
        # Print the currently running script
        echo "Currently running: $script"

        # Execute the script in the background, ignore errors, and save the output to a temporary file
        output_file=$(mktemp)
        bash "$script" 2>&1 > "$output_file" || handle_errors &

        # Get the process ID of the background job
        script_pid=$!

        # Continuously print the last five lines of the output while the script is running
        while kill -0 $script_pid 2>/dev/null; do
            clear
            echo "Currently running: $script"
            echo "Scheduled jobs:"
            for s in $scripts; do
                echo "  - $s"
            done
            echo ""
            echo "Output:"
            tail -n 20 "$output_file"
            sleep 1
        done

        # Wait for the background job to finish and retrieve its exit status
        wait $script_pid
        exit_status=$?

        # Handle errors if the exit status is non-zero
        if [ $exit_status -ne 0 ]; then
            handle_errors
        fi

        # Remove the temporary output file
        rm "$output_file"

        # Move the script to the "completed" folder
        mv "$script" completed/

        # Print that the script has completed
        echo "Completed: $script"
        echo ""
    done

    # Recheck for new scripts
done
