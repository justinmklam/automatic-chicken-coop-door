/**
 * Announcer for PC Serial output
 */
void announcePos(int position)
{
  Serial.print("Positiion set to ");
  Serial.println(position);
  Serial.flush();
}

/**
 * Reads serial and loads data to array
 */
void loadBufferFromSerial(char *buffer)
{
  uint8_t pointer = 0;

  while (pointer < BUFFER_SIZE - 1)
  {                                  // accumulate 4 chars
    buffer[pointer] = Serial.read(); // store in the buffer
    pointer++;                       // move the pointer forward by 1
  }

  Serial.flush();
}

/**
 * Converts buffer to integer
 */
uint16_t stringBufferToInt(char buffer[])
{
  uint16_t output = 0;

  output = (buffer[0] - 48) * 1000 + (buffer[1] - 48) * 100 + (buffer[2] - 48) * 10 + (buffer[3] - 48);

  return output;
}
