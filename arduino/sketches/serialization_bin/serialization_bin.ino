/*
 * Serialización usando datos binarios
 * 
 */

uint8_t buf[10];

uint8_t float_serialize(uint8_t *outbuffer, const float& data)
{
  uint8_t offset = 0;
  union {
    float real;
    uint32_t base;
  } u_data;
  u_data.real = data;
  *(outbuffer + offset + 0U) = (u_data.base >> (8U * 0U)) & 0xFF;
  *(outbuffer + offset + 1U) = (u_data.base >> (8U * 1U)) & 0xFF;
  *(outbuffer + offset + 2U) = (u_data.base >> (8U * 2U)) & 0xFF;
  *(outbuffer + offset + 3U) = (u_data.base >> (8U * 3U)) & 0xFF;
  offset += sizeof(data);
  return offset;
}


void setup()
{   
  Serial.begin(9600); 
}

void loop()
{
  const float pif = 3.141592741012573242187f;
  uint8_t size = float_serialize(buf, pif);
  // Header
  Serial.write(0xFF);
  Serial.write(0xFF);
  // Data
  Serial.write(buf, size);
  // Tail
  Serial.write(0x00);
  delay(100);
}