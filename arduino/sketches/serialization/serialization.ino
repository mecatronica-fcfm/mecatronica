/*
 * Serialización usando datos en formato ASCII.
 * 
 */


void setup()
{   
  Serial.begin(9600); 
}

void loop()
{
  // Leer A0 (0-255 -> 0-5v)
  int value = analogRead(A0);

  /*
  * La función Serial.print se encaga de transformar
  * números a su representación como cadena de caracteres
  * 
  * Incluir carácteres especiales al inicio y final
  * puede ayudar a la detección de errores
  *
  */

  Serial.print('{'); // Cabezera (header)
  Serial.print(value); // Primer valor
  Serial.print(','); // Separador
  Serial.print(millis());
  Serial.println('}'); // eq. Serial.print("}\r\n")

  delay(100);

}
