/* * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*  KALMAN FILTER IN C + asm
*  AUTHOR: ROBERT VALNER
*  2016
*
*  This is a testing code for kalman filter, that
*  is implemented in C and asm where asm part
*  executes the filtering process. Input data,
*  which is used for simulation, is gathered from  
*  an inertial measurement unit (IMU) that I built.
*  This IMU runs c++ based kalman filter code.
*
*	Currently there is no static C library because of lack of time
*	and therefore lack of knowldedge about makefiles.
*
*  MORE INFO ON IMU: https://protosity.wordpress.com/2015/08/17/inertial-measurement-unit-software-development/
*
*  Credits again to Lauszuz who provided the basecode
*  Lauszuz´s blog post: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "KalmanFilter.h"

// Function that checks basic input file errors. Returns file handle
FILE* inputfileControl(int argc, char* argv[])
{
    // Check it there is enough input arguments
    if (argc < 2)
    {
        printf("Error: File name not entered\nTry 'KalmanFilter sampleData.txt' \n");
        return NULL;
    }

    // Open the file
    FILE* file = fopen(argv[1], "r");

    // Check if the path is valid
    if(file == NULL)
    {
        printf("Error: No such file or directory\n");
        return NULL;
    }

    // Check if there is any content in the file
    struct stat s;
    fstat(fileno(file), &s);

    if(s.st_size < 1)
    {
        printf("Error: Empty file\n");
        return NULL;
    }

    return file;
}

int main(int argc, char* argv[])
{
    FILE* file = inputfileControl(argc, argv);

    if (file != NULL)
    {
		printf("------------------------------------------------\n\n");
		printf("            KALMAN FILTER IN C + asm\n");
		printf("            AUTHOR: ROBERT VALNER\n\n");
		printf("---This is a testing code for kalman filter, that  ---\n");
		printf("---is implemented in C and asm where asm part	   ---\n");
		printf("---executes the filtering process. Input data,	   ---\n");
		printf("---which is used for simulation, is gathered from  ---\n");
		printf("---an inertial measurement unit (IMU) that I built.---\n");
		printf("---This IMU runs c++ based kalman filter code	   ---\n");
		printf("   more info: \n");
		printf("https://protosity.wordpress.com/2015/08/17/inertial-measurement-unit-software-development/ \n\n");
		
        // Construct KalmanFilter Structure
        KalmanFilter* filter = kalmanFilter_Construct(0.09, 0.003, 0.01, 0.1);

        // Data storage arrays. Currently basically unused, left in for debugging purposes
        float* accelerometerData;
        float* gyroData;
        float* kalmanData;

        char line[30];  // Array that is used to manipulate with input file data
        int lines = 0;  // Number of lines in the input file

        // Get the number of lines in the input txt file
        while (fgets(line, sizeof(line), file))
            lines++;
		
        printf("* Lines in input file: %d \n", lines);

        //Allocate memory to store data
        accelerometerData = malloc(lines * sizeof(float));
        gyroData          = malloc(lines * sizeof(float));
        kalmanData        = malloc(lines * sizeof(float));

        // Go back to the beginning of the file
        fseek(file, 0, SEEK_SET);

        // Insert the input data into according arrays
        printf("* Copying txt data to storage arrays \n");
        int i = 0;

        while (fgets(line, sizeof(line), file) || (i < lines))
        {
            // Expected format for the "line" is "%f %f %f"
            if(sscanf(line, "%f %f %f", &accelerometerData[i], &gyroData[i], &kalmanData[i]) == 3)
            {
                i++;
            }
            else
            {
                printf("\nError: Input in unexpected format at line %d in %s\nFilter simulation is executed with current valid %d inputs\n\n", i+1, argv[1], i);
                lines = i;
                break;
            }
        }
        /*
        Initialize Kalman filter. Should be carefully measured in standstill.
        I currently use the first values from the input sample data file.
        This causes the reference Kalman filter data to differ from the data
        that is calculated with this code until filter stabilizes.
        */
        kalmanFilter_Init(filter, gyroData[0], accelerometerData[0]);

        // print out the values
        printf("* Starting Kalman filter simulation.\n\n");
		printf("  Calc Kalman row is the output of the simulation \n");
		printf("  based on premeasured accelerometer & gyro data.  \n");
		printf("  Ref Tilt is precalculated (given in input file)\n");
		printf("  valid kalman filter output. So Calc Tilt Kalman and \n");
		printf("  Ref Tilt have to be more or less equal after the filter stabilises\n\n");
		
        printf("Accelerometer  | Gyro  |  Ref Tilt    |  Calc Tilt Kalman\n");
        printf("    Deg        | Deg/s |    Deg       |      Deg\n");
        printf("----------------------------------------------------\n");
        for(i = 0; i < lines; i++)
        {
            // Run the filter
            kalmanFilter_Iterate(filter, &gyroData[i], &accelerometerData[i]);

            // Print out values
            printf("%10.2f | %9.2f | %12.2f | %10.2f\n",accelerometerData[i], gyroData[i], kalmanData[i], filter->angle);
        }

        // Close file and destroy Kalman filter
        fclose(file);
        kalmanFilter_Destroy(filter); // make sure to free any allocated memory
    }

    return 0;
}
