#include <Vector.h>
#include <StackArray.h>
#include <Wire.h>
#include <MPU6050_light.h>

const int ENA = 7; //means pin 9 on the Arduino controls the speed of left motor
const int ENB = 8;
const int IN1 = 3; //left 1 and left 2 control the direction of rotation of left motor
const int IN2 = 4;
const int IN3 = 6;
const int IN4 = 5;
const int trigPin1 = 9;
const int echoPin1 = A0;
const int trigPin2 = 11;
const int echoPin2 = A2;
const int trigPin3 = 10;
const int echoPin3 = A1;
const int walldistance = 20;
int InitialGyro = 0;
int angleRotationSpeed = 30;
int x = 0;
Vector<Vector<int>> vc;
#define EncoderPin 2
#define EncoderPin2 12
MPU6050 mpu(Wire);
unsigned long timer = 0;
int gyro = 0;
int stat = 0;
long duration;
int distance;
long duration2;
int distance2;
long duration3;
int distance3;
typedef struct cell
{
  bool FrontWall;
  bool LeftWall;
  bool RightWall;
  int VisitedCount;
} Cell;
int currentX = 0;
int currentY = 0;
int CenterX = 0;
int Centery = 0;
char currentDir = 'F';
char latestDir = 'N';
Cell Maze[5][5] {{0}};
int row[] = { -1, 0, 0, 1 };
int col[] = { 0, -1, 1, 0 };
int INT_MAX = 2147483646;
class Cellx {
  public:
    int row;
    int col;

    Cellx(int _row, int _col) {
      row = _row;
      col = _col;
    }
};

bool isValid(int row, int col, int maxRow, int maxCol) {
  return row >= 0 && row < maxRow && col >= 0 && col < maxCol;
}

void addNeighbors(Cellx cell, Vector<Cellx> &cells, int maxRow, int maxCol) {
  int ds[4][2] = {{ -1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int i = 0; i < 4; i++) {
    int row = cell.row + ds[i][0];
    int col = cell.col + ds[i][1];
    if (isValid(row, col, maxRow, maxCol))
      cells.push_back(Cellx(row, col));
  }
}

Cellx getNeighbor(Cellx cell, int distance, Vector<Vector<int>> distances) {
  int ds[4][2] = {{ -1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int i = 0; i < 4; i++) {
    int row = cell.row + ds[i][0];
    int col = cell.col + ds[i][1];
    if (isValid(row, col, distances.size(), distances[0].size()) && distances[row][col] == distance)
      return Cellx(row, col);
  }
  return Cellx(0, 0);
}

int GetShortestPath(Vector<Vector<int>> mat, Cellx start, Cellx dist, StackArray<Cellx> &path)
{
  Vector<Vector<int>> distances;
  for (int i = 0; i < 16; i++)
  {
    for (int j = 0; j < 16; j++)
    {
      distances[i][j] = INT_MAX;
    }
  }
  int distance = 0;
  Vector<Cellx> currentCells;
  currentCells.push_back(start);
  while (distances[dist.row][dist.col] == INT_MAX && currentCells.size() != 0)
  {
    if (distances[dist.row][dist.col] != INT_MAX)
    {
      break;
    }
    Vector<Cellx> nextCells;
    for (int i = 0; i < currentCells.size(); i++)
    {
      int currRow = currentCells[i].row;
      int currCol = currentCells[i].col;
      if (distances[currRow][currCol] == INT_MAX && mat[currRow][currCol] != 1)
      {
        distances[currRow][currCol] = distance;
        addNeighbors(currentCells[i], nextCells, mat.size(), mat[0].size());
      }
    }
    currentCells = nextCells;
    distance++;
  }

  if (distances[dist.row][dist.col] < INT_MAX)
  {
    Cellx cell = dist;
    path.push(dist);
    for (int d = distances[dist.row][dist.col] - 1; d >= 0; d--)
    {
      cell = getNeighbor(cell, d, distances);
      path.push(cell);
    }
  }

  return distances[dist.row][dist.col];

}

float ultraFront() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin1, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back

  return distance;
}


float ultraRight() {
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration2 = pulseIn(echoPin3, HIGH);
  distance2 = duration2 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance2;
}


float ultraLeft() {
  int average = 0;
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration3 = pulseIn(echoPin2, HIGH);
  distance3 = duration3 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance3;
}

void right2() {
  mpu.update();
  int constGyro = mpu.getAngleZ() + 90;

   
  while (mpu.getAngleZ() < constGyro - 0.5 || mpu.getAngleZ() > constGyro + 0.5)
  {
    mpu.update();

    if (mpu.getAngleZ() < constGyro - 0.5)
    {
      int errorP = abs(abs(mpu.getAngleZ()) - abs(constGyro));
      if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5)
      {
        errorP *= 6;  
      }
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, angleRotationSpeed + errorP);
      analogWrite(ENB, angleRotationSpeed + errorP);
    }
    else if (mpu.getAngleZ() > constGyro + 0.5)
    {
      int errorP = abs(abs(constGyro) - abs(mpu.getAngleZ()));
      if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5)
      {
        errorP *= 6;  
      }
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, angleRotationSpeed + errorP);
      analogWrite(ENB, angleRotationSpeed + errorP);
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    }
    delay(40);
  }
  delay(500);
}

void right()
{
  mpu.update();
  float constGyro = mpu.getAngleZ() - 90;
  while (mpu.getAngleZ() > constGyro + 0.5 || mpu.getAngleZ() < constGyro - 0.5)
  {
    mpu.update();

    if (mpu.getAngleZ() > constGyro + 0.5)
    {
      int errorP = abs(abs(constGyro) - abs(mpu.getAngleZ()));
            if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5 && errorP > 2)
      {
        errorP *= 6;  
      }

      else if(errorP <= 2)
      {
        errorP *= 15;  
      }

            digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, angleRotationSpeed + errorP);
     
    }
    else if (mpu.getAngleZ() < constGyro - 0.5)
    {
      int errorP = abs(abs(mpu.getAngleZ()) - abs(constGyro));
      if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5 && errorP > 2)
      {
        errorP *= 6;  
      }

      else if(errorP <= 2)
      {
        errorP *= 10;  
      }
       digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, angleRotationSpeed + errorP);
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    }
    delay(80);
  }
  delay(500);

}

void left() {
  mpu.update();
  float constGyro = mpu.getAngleZ() + 90;
  while (mpu.getAngleZ() < constGyro - 0.5 || mpu.getAngleZ() > constGyro + 0.5)
  {
    mpu.update();

    if (mpu.getAngleZ() < constGyro - 0.5)
    {
      int errorP = abs(abs(constGyro) - abs(mpu.getAngleZ()));
            if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5 && errorP > 2)
      {
        errorP *= 6;  
      }

      else if(errorP <= 2)
      {
        errorP *= 10;  
      }
      
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, angleRotationSpeed + errorP);
    }
    else if (mpu.getAngleZ() > constGyro + 0.5)
    {
      int errorP = abs(abs(mpu.getAngleZ()) - abs(constGyro));
      if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5 && errorP > 2)
      {
        errorP *= 6;  
      }

      else if(errorP <= 2)
      {
        errorP *= 15;  
      }
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, angleRotationSpeed + errorP);
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    }
    delay(80);
  }
  delay(500);

}

void BackwardRotation() {
  mpu.update();
  float constGyro = mpu.getAngleZ() - 180;
  while (mpu.getAngleZ() > constGyro + 0.5 || mpu.getAngleZ() < constGyro - 0.5)
  {
    mpu.update();

    if (mpu.getAngleZ() > constGyro + 0.5)
    {
      int errorP = abs(abs(constGyro) - abs(mpu.getAngleZ()));
            if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5 && errorP > 2)
      {
        errorP *= 6;  
      }

      else if(errorP <= 2)
      {
        errorP *= 15;  
      }

      digitalWrite(IN1, LOW);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, angleRotationSpeed + errorP);
      analogWrite(ENB, angleRotationSpeed + errorP);
     
    }
    else if (mpu.getAngleZ() < constGyro - 0.5)
    {
      int errorP = abs(abs(mpu.getAngleZ()) - abs(constGyro));
      if(errorP < 20 && errorP > 5)
      {
        errorP *= 2;
      }
      else if(errorP <= 5 && errorP > 2)
      {
        errorP *= 6;  
      }

      else if(errorP <= 2)
      {
        errorP *= 10;  
      }
      digitalWrite(IN1, HIGH);
      digitalWrite(IN1, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, angleRotationSpeed + errorP);
      analogWrite(ENB, angleRotationSpeed + errorP);
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    }
    delay(80);
  }
  delay(500);

}

bool wallLeft() {
  if ((ultraLeft() <= 20))
    return true;
  else return false;
}
bool wallRight() {
  if ((ultraRight() <= 20))
    return true;
  else return false;
}
bool wallFront() {
  // || ultraFront() > 1000
  if ((ultraFront() <= 15))
    return true;
  else return false;
}



void MoveForward(){
  volatile int count = 0;
  byte lastValue = digitalRead(EncoderPin);
  mpu.update();
  int sign;
  
  if(mpu.getAngleZ() < -30)
  {
    sign = -1;  
  }
  else
  {
    sign = 1;  
  }

  if(abs(mpu.getAngleZ()) < 50)
  {
    gyro = 0;  
  }
  else if(abs(mpu.getAngleZ()) > 50 && abs(mpu.getAngleZ()) < 160)
  {
    gyro = 90 * sign;  
  }

  else
  {
    gyro = 180 * sign;  
  }
  while (true)
  {
    mpu.update();
    if (count >= 10)
    {
      break;
    }
    if(ultraFront() <= 10){
      break;
    }
    if (digitalRead(EncoderPin) != lastValue)
    {
      lastValue = digitalRead(EncoderPin);
      count += 1;
    }
    int err;
    int val = 60;

    if(abs(abs(mpu.getAngleZ()) - abs(gyro)) >= 20)
    {
      err = abs(abs(mpu.getAngleZ()) - abs(gyro));
    }
    else
    {
      err = abs(abs(mpu.getAngleZ()) - abs(gyro)) * 3;
    }
    if (mpu.getAngleZ() < gyro) {
      analogWrite(ENA, val + err);
      analogWrite(ENB, val - err);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    else if (mpu.getAngleZ() > gyro) {
      analogWrite(ENA, val - err);
      analogWrite(ENB, val + err);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    else {
      analogWrite(ENA, val);
      analogWrite(ENB, val);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    int er = 40;
    int cright = ultraRight();
    int cleft = ultraLeft();

    if(ultraRight() >= 1000)
    {
      cright = 2; 
    }
    if(ultraLeft() >= 1000)
    {
      cleft = 2; 
    }

    if ((ultraRight() > ultraLeft())&&(ultraLeft() < 24)&&(ultraRight() < 24)) {
      analogWrite(ENA, val - er);
      analogWrite(ENB, val + er);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      delay(10);
    }
    else if ((ultraRight() < ultraLeft())&&(ultraRight() < 24)&&(ultraLeft() < 24)) {
      analogWrite(ENA, val + er);
      analogWrite(ENB, val - er);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      delay(10);
    }
    else {
      analogWrite(ENA, val);
      analogWrite(ENB, val);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(10);
  
}
void MoveInDirection(char dir)
{

  if (dir == 'F')
  {
    if (currentDir == 'F')
    {
      currentY += 1;
      MoveForward();
    }

    else if (currentDir == 'R')
    {
      currentX += 1;
      MoveForward();
    }

    else if (currentDir == 'L')
    {
      currentX -= 1;
      MoveForward();
    }

    else if (currentDir == 'B')
    {
      currentY -= 1;
      MoveForward();
    }
  }

  else if (dir == 'R')
  {
    right();
    if (currentDir == 'F')
    {
      currentX += 1;
      currentDir = 'R';
      MoveForward();
    }

    else if (currentDir == 'R')
    {
      currentY -= 1;
      currentDir = 'B';
      MoveForward();
    }

    else if (currentDir == 'L')
    {
      currentY += 1;
      currentDir = 'F';
      MoveForward();
    }

    else if (currentDir == 'B')
    {
      currentX -= 1;
      currentDir = 'L';
      MoveForward();
    }
  }

  else if (dir == 'L')
  {
    left();
    if (currentDir == 'F')
    {
      currentX -= 1;
      currentDir = 'L';
      MoveForward();
    }

    else if (currentDir == 'R')
    {
      currentY += 1;
      currentDir = 'F';
      MoveForward();
    }

    else if (currentDir == 'L')
    {
      currentY -= 1;
      currentDir = 'B';
      MoveForward();
    }

    else if (currentDir == 'B')
    {
      currentX += 1;
      currentDir = 'R';
      MoveForward();
    }
  }

  else if (dir == 'B')
  {
    BackwardRotation();
    if (currentDir == 'F')
    {
      currentY -= 1;
      currentDir = 'B';
      MoveForward();
    }

    else if (currentDir == 'R')
    {
      currentX -= 1;
      currentDir = 'L';
      MoveForward();
    }

    else if (currentDir == 'L')
    {
      currentX += 1;
      currentDir = 'R';
      MoveForward();
    }

    else if (currentDir == 'B')
    {
      currentY += 1;
      currentDir = 'F';
      MoveForward();
    }
  }

  if (Maze[currentX][currentY].VisitedCount == 0)
  {
    if (currentDir == 'F')
    {
      Maze[currentX][currentY].FrontWall = wallFront();
      Maze[currentX][currentY].LeftWall = wallLeft();
      Maze[currentX][currentY].RightWall = wallRight();
    }
    else if (currentDir == 'R')
    {
      Maze[currentX][currentY].FrontWall = wallLeft();
      Maze[currentX][currentY].LeftWall = 0;
      Maze[currentX][currentY].RightWall = wallFront();
    }
    else if (currentDir == 'L')
    {
      Maze[currentX][currentY].FrontWall = wallRight();
      Maze[currentX][currentY].RightWall = 0;
      Maze[currentX][currentY].LeftWall = wallFront();
    }
    else if (currentDir == 'B')
    {
      Maze[currentX][currentY].FrontWall = 0;
      Maze[currentX][currentY].RightWall = wallLeft();
      Maze[currentX][currentY].LeftWall = wallRight();
    }

  }
  Maze[currentX][currentY].VisitedCount += 1;
}

char GetLeastVisitedPath(char currDir)
{
  if (wallFront() && wallRight() && wallLeft())
  {
    Maze[currentX][currentY].VisitedCount = 125;
    return 'B';
  }
  if (currDir == 'F')
  {
    if ((wallLeft() || Maze[currentX - 1][currentY].VisitedCount == 123) && (wallRight() || Maze[currentX + 1][currentY].VisitedCount == 123) && !wallFront())
    {
      if (Maze[currentX][currentY + 1].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }
      else
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'F';
        }

        else
        {
          return 'B';
        }

      }
    }

    else if ((wallFront() || Maze[currentX][currentY + 1].VisitedCount == 123) && (wallLeft() || Maze[currentX - 1][currentY].VisitedCount == 123) && !wallRight())
    {
      if (Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'R';
        }

        else
        {
          return 'B';
        }
      }
    }

    else if ((wallRight() || Maze[currentX + 1][currentY].VisitedCount == 123) && !wallLeft() && (wallFront() || Maze[currentX][currentY + 1].VisitedCount == 123))
    {
      if (Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'L';
        }

        else
        {
          return 'B';
        }
      }
    }

    else
    {
      if ((!wallLeft() && Maze[currentX - 1][currentY].VisitedCount != 123) && (!wallRight() && Maze[currentX + 1][currentY].VisitedCount != 123) && (!wallFront() && Maze[currentX][currentY + 1].VisitedCount != 123))
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX + 1][currentY].VisitedCount && Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX - 1][currentY].VisitedCount)
        {
          return 'F';
        }

        else if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX - 1][currentY].VisitedCount)
        {
          return 'R';
        }


        else
        {
          return 'L';
        }
      }

      else if (wallLeft() || Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX + 1][currentY].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'R';
        }
      }

      else if (wallRight() || Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX - 1][currentY].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallFront() || Maze[currentX][currentY + 1].VisitedCount == 123)
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX - 1][currentY].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else
      {
        return 'F';
      }
    }

  }

  else if (currDir == 'R')
  {
    if ((wallLeft() || Maze[currentX][currentY + 1].VisitedCount == 123) && (wallRight() || Maze[currentX][currentY - 1].VisitedCount == 123) && !wallFront())
    {
      if (Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }
      else
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'F';
        }

        else
        {
          return 'B';
        }

      }
    }

    else if ((wallFront() || Maze[currentX + 1][currentY].VisitedCount == 123) && (wallLeft() || Maze[currentX][currentY + 1].VisitedCount == 123) && !wallRight())
    {
      if (Maze[currentX][currentY - 1].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'R';
        }

        else
        {
          return 'B';
        }
      }
    }

    else if ((wallRight() || Maze[currentX][currentY - 1].VisitedCount == 123) && !wallLeft() && (wallFront() || Maze[currentX + 1][currentY].VisitedCount == 123))
    {
      if (Maze[currentX][currentY + 1].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'L';
        }

        else
        {
          return 'B';
        }
      }
    }

    else
    {
      if ((!wallLeft() && Maze[currentX][currentY + 1].VisitedCount != 123) && (!wallRight() && Maze[currentX][currentY - 1].VisitedCount != 123) && (!wallFront() && Maze[currentX + 1][currentY].VisitedCount != 123))
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY - 1].VisitedCount && Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY + 1].VisitedCount)
        {
          return 'F';
        }
        else if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX][currentY + 1].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallLeft() || Maze[currentX][currentY + 1].VisitedCount == 123)
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY - 1].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'R';
        }
      }

      else if (wallRight() || Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY + 1].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallFront() || Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX][currentY + 1].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else
      {
        return 'F';
      }
    }

  }

  else if (currDir == 'L')
  {
    if ((wallLeft() || Maze[currentX][currentY - 1].VisitedCount == 123) && (wallRight() || Maze[currentX][currentY + 1].VisitedCount == 123) && !wallFront())
    {
      if (Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }
      else
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'F';
        }

        else
        {
          return 'B';
        }

      }
    }

    else if ((wallFront() || Maze[currentX - 1][currentY].VisitedCount == 123) && (wallLeft() || Maze[currentX][currentY - 1].VisitedCount == 123) && !wallRight())
    {
      if (Maze[currentX][currentY - 1].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'R';
        }

        else
        {
          return 'B';
        }
      }
    }

    else if ((wallRight() || Maze[currentX][currentY + 1].VisitedCount == 123) && !wallLeft() && (wallFront() || Maze[currentX - 1][currentY].VisitedCount == 123))
    {
      if (Maze[currentX][currentY - 1].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'L';
        }

        else
        {
          return 'B';
        }
      }
    }

    else
    {
      if ((!wallLeft() && Maze[currentX][currentY - 1].VisitedCount != 123) && (!wallRight() && Maze[currentX][currentY + 1].VisitedCount != 123) && (!wallFront() && Maze[currentX - 1][currentY].VisitedCount != 123))
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY + 1].VisitedCount && Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY - 1].VisitedCount)
        {
          return 'F';
        }
        else if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX][currentY - 1].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallLeft() || Maze[currentX][currentY - 1].VisitedCount == 123)
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY + 1].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'R';
        }
      }

      else if (wallRight() || Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY - 1].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallFront() || Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX][currentY + 1].VisitedCount <= Maze[currentX][currentY - 1].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else
      {
        return 'F';
      }
    }
  }

  else //currDir == 'B'
  {
    if ((wallLeft() || Maze[currentX + 1][currentY].VisitedCount == 123) && (wallRight() || Maze[currentX - 1][currentY].VisitedCount == 123) && !wallFront())
    {
      if (Maze[currentX][currentY - 1].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }
      else
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'F';
        }

        else
        {
          return 'B';
        }

      }
    }

    else if ((wallFront() || Maze[currentX][currentY - 1].VisitedCount == 123) && (wallLeft() || Maze[currentX + 1][currentY].VisitedCount == 123) && !wallRight())
    {
      if (Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'R';
        }

        else
        {
          return 'B';
        }
      }
    }

    else if ((wallRight() || Maze[currentX - 1][currentY].VisitedCount == 123) && !wallLeft() && (wallFront() || Maze[currentX][currentY - 1].VisitedCount == 123))
    {
      if (Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        //Maze[currentX][currentY].VisitedCount = 123;
        return 'B';
      }

      else
      {
        if (Maze[currentX + 1][currentY].VisitedCount <= Maze[currentX][currentY].VisitedCount)
        {
          return 'L';
        }

        else
        {
          return 'B';
        }
      }
    }

    else
    {
      if ((!wallLeft() && Maze[currentX + 1][currentY].VisitedCount != 123) && (!wallRight() && Maze[currentX - 1][currentY].VisitedCount != 123) && (!wallFront() && Maze[currentX][currentY - 1].VisitedCount != 123))
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX - 1][currentY].VisitedCount && Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX + 1][currentY].VisitedCount)
        {
          return 'F';
        }
        else if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX + 1][currentY].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallLeft() || Maze[currentX + 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX - 1][currentY].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'R';
        }
      }

      else if (wallRight() || Maze[currentX - 1][currentY].VisitedCount == 123)
      {
        if (Maze[currentX][currentY - 1].VisitedCount <= Maze[currentX + 1][currentY].VisitedCount)
        {
          return 'F';
        }
        else
        {
          return 'L';
        }
      }

      else if (wallFront() || Maze[currentX][currentY - 1].VisitedCount == 123)
      {
        if (Maze[currentX - 1][currentY].VisitedCount <= Maze[currentX + 1][currentY].VisitedCount)
        {
          return 'R';
        }
        else
        {
          return 'L';
        }
      }

      else
      {
        return 'F';
      }
    }

  }
}

bool AllCellsVisited()
{
  for (int i = 0; i < 16; i++)
  {
    for (int j = 0; j < 16; j++)
    {
      if (Maze[i][j].VisitedCount == 0)
      {
        return false;
      }
    }
  }
  return true;
}

void Tremax()
{
  //(currentX != 8 && currentY != 8) || (currentX != 7 && currentY != 8) || (currentX != 7 && currentY != 7) || (currentX != 8 && currentY != 7)
  Maze[currentX][currentY].FrontWall = wallFront();
  Maze[currentX][currentY].LeftWall = wallLeft();
  Maze[currentX][currentY].RightWall = wallRight();
  while (AllCellsVisited() == false)
  {
    if (AllCellsVisited() == true)
    {
      break;
    }

    MoveInDirection(GetLeastVisitedPath(currentDir));
    delay(1000);
  }
}
void mapping() {
  Tremax();
  for (int i = 0; i < 16; i++)
  {
    Vector<int> compiledStringLeft;
    Vector<int> compiledStringUp;
    for (int j = 0; j < 16; j++)
    {
      compiledStringLeft.push_back(0);
      compiledStringLeft.push_back(Maze[j][i].RightWall);

      compiledStringUp.push_back(Maze[j][i].FrontWall);

      compiledStringUp.push_back(1);

    }

    vc.push_back(compiledStringLeft);
    vc.push_back(compiledStringUp);
  }
}


Cellx findend(Vector<Vector<int>> mat) {
  Cellx cell(0, 0);
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      if ((mat[i][j] == 1) && (mat[i][j + 1] == 0) && (mat[i + 1][j + 1] == 0) && (mat[i + 1][j] == 0) && (mat[i + 1][j - 1] == 0) && (mat[i][j - 1] == 0) && (mat[i - 1][j - 1] == 0) && (mat[i - 1][j] == 0) && (mat[i - 1][j + 1] == 0))
      {
        return Cellx(i - 1, j - 1);
      }
    }
  }
  return cell;
}


void LeftWallFollower()
{
  if(ultraLeft() <= 15 && ultraFront() >= 16)
  {
    MoveForward();
  }
  else if(ultraLeft() <= 15 && ultraFront() <= 16)
  {
    right();  
    MoveForward();
  }

  else
  {
    left(); 
    MoveForward(); 
  }
}

void Solve()
{

  mapping();
  while ((wallFront() || wallRight() || wallLeft()) == 1)
  {
  }

  //shortpathlength();
  StackArray<Cellx> path;
  Cellx start(0, 0);
  Cellx endc = findend(vc);
  int shortestpath = GetShortestPath(vc, start, endc, path);
  int lastX = 0;
  int lastY = 0;
  if (!wallFront())
  {
    currentDir = 'F';
  }
  else if (!wallRight())
  {
    currentDir = 'R';
    right();
  }
  path.pop();
  while (!path.isEmpty())
  {
    Cellx cell1 = path.pop();
    lastX = cell1.col;
    lastY = cell1.row;
    Cellx cell2 = path.pop();
    if (lastX == cell2.col)
    {
      if (cell2.row > lastY)
      {
        if (currentDir == 'F')
        {
          MoveInDirection('F');
        }
        else if (currentDir == 'L')
        {
          MoveInDirection('R');
        }
        else if (currentDir == 'R')
        {
          MoveInDirection('L');
        }
      }
      else
      {
        if (currentDir == 'B')
        {
          MoveInDirection('F');
        }
        else if (currentDir == 'L')
        {
          MoveInDirection('L');
        }
        else if (currentDir == 'R')
        {
          MoveInDirection('R');
        }
      }

    }
    else
    {
      if (cell2.col > lastX)
      {
        if (currentDir == 'R')
        {
          MoveInDirection('F');
        }
        else if (currentDir == 'F')
        {
          MoveInDirection('R');
        }
        else if (currentDir == 'B')
        {
          MoveInDirection('L');
        }
      }
      else
      {
        if (currentDir == 'L')
        {
          MoveInDirection('F');
        }
        else if (currentDir == 'F')
        {
          MoveInDirection('L');
        }
        else if (currentDir == 'B')
        {
          MoveInDirection('R');
        }
      }

    }
  }
}



void setup() {
  Serial.begin(9600);                           // Ensure serial monitor set to this value also
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin3, INPUT);
  pinMode(EncoderPin, INPUT);
  Solve();
}
void loop() {
  
} 
