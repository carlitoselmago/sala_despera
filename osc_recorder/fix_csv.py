import csv

def process_csv(input_file):
    # Define the output filename with '_fixed' prefix
    output_file = "_fixed_" + input_file

    with open(input_file, mode='r', newline='', encoding='utf-8') as infile:
        # Reading the CSV file
        reader = csv.reader(infile)
        
        with open(output_file, mode='w', newline='', encoding='utf-8') as outfile:
            # Writing the processed CSV with semicolon as the separator
            writer = csv.writer(outfile, delimiter=';')

            for row in reader:
                # Write each row using semicolons between columns
                writer.writerow(row)

    print(f"File saved as {output_file}")

# Example usage:
input_file = 'osc_messages.csv'  # Replace with your actual file name
process_csv(input_file)
