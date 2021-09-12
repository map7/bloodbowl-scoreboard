#include <Colorduino.h>
void setup()
{
  Colorduino.Init();
  
  
  for (int i=0; i<=8; i++)
  {
    Colorduino.SetPixel(1, i, 200, 200, 0); // set the pixel at 3, 5 to have a bluish color
  }
  
  Colorduino.FlipPage(); // swap screen buffers to show it
}

void loop()
{

}
