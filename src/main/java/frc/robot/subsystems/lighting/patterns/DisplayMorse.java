package frc.robot.subsystems.lighting.patterns;

import java.util.ArrayList;
import java.util.HashMap;

import frc.robot.Constants;
import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class DisplayMorse extends LEDPattern {
  HashMap<Character, String> morseMap = new HashMap<>();

  String message;
  String morseString;
  boolean[] messageArray;
  ArrayList<Boolean> arr;

  public DisplayMorse(String message) {
    super(6, "DisplayMorse");
    morseMap.put('a', ".-");
    morseMap.put('b', "-...");
    morseMap.put('c', "-.-.");
    morseMap.put('d', "-..");
    morseMap.put('e', ".");
    morseMap.put('f', "..-.");
    morseMap.put('g', "--.");
    morseMap.put('h', "....");
    morseMap.put('i', "..");
    morseMap.put('j', ".---");
    morseMap.put('k', "-.-");
    morseMap.put('l', ".-..");
    morseMap.put('m', "--");
    morseMap.put('n', "-.");
    morseMap.put('o', "---");
    morseMap.put('p', ".--.");
    morseMap.put('q', "--.-");
    morseMap.put('r', ".-.");
    morseMap.put('s', "...");
    morseMap.put('t', "-");
    morseMap.put('u', "..-");
    morseMap.put('v', "...-");
    morseMap.put('w', ".--");
    morseMap.put('x', "-..-");
    morseMap.put('y', "-.--");
    morseMap.put('z', "--..");
    morseMap.put('1', ".----");
    morseMap.put('2', "..---");
    morseMap.put('3', "...--");
    morseMap.put('4', "....-");
    morseMap.put('5', ".....");
    morseMap.put('6', "-....");
    morseMap.put('7', "--...");
    morseMap.put('8', "---..");
    morseMap.put('9', "----.");
    morseMap.put('0', "-----");
    this.message = message + " ";
    this.morseString = textToMorseString();
    arr = new ArrayList<>();
    for (char c : morseString.toCharArray()) {
      if (c == '.') {
          arr.add(true);
          arr.add(false);
      } else if (c == '-') {
          arr.add(true);
          arr.add(true);
          arr.add(true);
          arr.add(false);
      } else if (c == ' ') {
          arr.add(false);
          arr.add(false);
      } else if (c == '/') {
          arr.add(false);
          arr.add(false);
          arr.add(false);
          arr.add(false);
          arr.add(false);
          arr.add(false);
      }
    }
  }

  @Override
  public void setPixels() {
    if (arr.get(tick % getMorseCodeLength())) {
      on();
    } else {
      off();
    }
  }

  public String textToMorseString() {
    StringBuilder sb = new StringBuilder();
    boolean addSpace = false;
    for (char c : message.toCharArray()) {
      String morse = morseMap.get(Character.toLowerCase(c));
      if (morse != null) {
        if (addSpace) {
          sb.append(" ");
          addSpace = false;
        }
        sb.append(morse).append(" ");
      } else if (c == ' ') {
        sb.append("/");
        addSpace = true;
      }
    }
    return sb.toString().replaceAll("/ ", "/").replaceAll(" /", "/");
  }

  public int getMorseCodeLength() {
    int units = 0;
    for (char c : morseString.toCharArray()) {
      if (c == '.') {
        units += 2;
      } else if (c == '-') {
        units += 4;
      } else if (c == ' ') {
        units += 2;
      } else if (c == '/') {
        units += 6;
      }
    }
    return units;
  }

  public void off() {
    LEDStrip.setAllRGB(0, 0, 0);
  }

  public void on() {
    LEDStrip.setAllRGB(255, 0, 0);
  }
}
