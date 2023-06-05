#include <MAX6675.h>
#include <PID_v1.h>
#include <scpiparser.h>
#include <Arduino.h>

#define CS_PIN 10
#define ALPHA 0.22

bool output_enable;
double setpoint, temperature, output;
char line_buffer[256];
uint8_t P, I, D;

/* initialize thermocouple IC with hardware spi */
MAX6675 tcouple(CS_PIN);

/* create PID controller with proportional on measurement mode */
PID myPID(&temperature, &output, &setpoint, 2, 5, 1, P_ON_M, DIRECT); 

/* declare scpi parser */
struct scpi_parser_context ctx;
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command);

scpi_error_t get_temperature(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_coefficients(struct scpi_parser_context* context, struct scpi_token* command);

scpi_error_t set_setpoint(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_setpoint(struct scpi_parser_context* context, struct scpi_token* command);

scpi_error_t enable(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t disable(struct scpi_parser_context* context, struct scpi_token* command);


void setup()
{
  struct scpi_command* controller;
  scpi_init(&ctx);

  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);
  controller = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "CONTROLLER", 10, "CTRL", 4, NULL);

  scpi_register_command(controller, SCPI_CL_CHILD, "TEMPERATURE?", 12, "TEMP?", 5, get_temperature);
  scpi_register_command(controller, SCPI_CL_CHILD, "COEFFICIENTS?", 13, "COEFF?", 6, get_coefficients);

  scpi_register_command(controller, SCPI_CL_CHILD, "SETPOINT", 8, "SET", 3, set_setpoint);
  scpi_register_command(controller, SCPI_CL_CHILD, "SETPOINT?", 9, "SET?", 4, get_setpoint);

  scpi_register_command(controller, SCPI_CL_CHILD, "ENABLE", 6, "ON", 2, enable);
  scpi_register_command(controller, SCPI_CL_CHILD, "DISABLE", 7, "OFF", 3, disable);

  
  
  Serial.begin(9600);
  
  /* dummy conversion */
  tcouple.readTempC();
  delay(500);
  
  /* first reading for exponential smoothing */
  temperature = tcouple.readTempC();
  delay(500);
}

/* loop forever */
void loop()
{
  uint8_t read_length;
  float reading = tcouple.readTempC();
  temperature = ALPHA * reading + (1.0 - ALPHA) * temperature;
  
  read_length = Serial.readBytesUntil('\n', line_buffer, 256);  
  if(read_length > 0)
  {
    scpi_execute_command(&ctx, line_buffer, read_length);
  }
  delay(500);
}


scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);

  Serial.println("OIC,Embedded SCPI Example,1,10");
  return SCPI_SUCCESS;
}


scpi_error_t set_setpoint(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, 0);
  
  if(output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'C'))
  {
    value = output_numeric.value;
  
    if(value < 0.0)
    {
      scpi_error error;
      error.id = -301;
      error.description = "Command error: Temperature below minimum";
      error.length = 40;
      scpi_queue_error(&ctx, error);
      scpi_free_tokens(command);
      return SCPI_SUCCESS;
    }

    else if(value > 600.0)
    {
      scpi_error error;
      error.id = -302;
      error.description = "Command error: Temperature above maximum";
      error.length = 40;
      scpi_queue_error(&ctx, error);
      scpi_free_tokens(command);
      return SCPI_SUCCESS;
    }

    else
    {
      setpoint = value;
      return SCPI_SUCCESS;
    }
  } 
  
  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}


scpi_error_t get_setpoint(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);
  Serial.println(setpoint, 1);
  return SCPI_SUCCESS;
}


scpi_error_t get_temperature(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);
  Serial.println(temperature, 1);
  return SCPI_SUCCESS;
}


scpi_error_t get_coefficients(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);
  Serial.print(P);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.println(D);
  return SCPI_SUCCESS;
}


scpi_error_t enable(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);
  output_enable = true;
  return SCPI_SUCCESS;
}


scpi_error_t disable(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);
  output_enable = false;
  return SCPI_SUCCESS;
}

