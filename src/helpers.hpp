#include <mbed.h>


uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
    //Subtract the oldest number from the prev sum, add the new number
    *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
    //Assign the nextNum to the position in the array
    ptrArrNumbers[pos] = nextNum;
    //return the average
    return *ptrSum / len;
};