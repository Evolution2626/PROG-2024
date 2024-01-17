package frc.util;

/** Classe avec plusieurs fonctions sur les ranges */
public class Range {

  /**
   * Retourne si le nombre est dans la range
   *
   * @param min
   * @param max
   * @param number
   * @return si le nombre est dans la range
   */
  public static boolean inRange(double min, double max, double number) {
    return (number >= min && number <= max);
  }

  /**
   * Contraint un nombre entre 2 valeurs
   *
   * @param min
   * @param max
   * @param number
   * @return le nombre contraint entre le min et le max
   */
  public static double coerce(double min, double max, double number) {
    if (number < min) return min;
    if (number > max) return max;
    return number;
  }

  public static double coerce(double max, double number) {
    return coerce(-max, max, number);
  }

  public static double minCoerce(double threshold, double number) {
    if (number == 0) return 0;
    if (inRange(-threshold, threshold, number)) {
      if (number > 0) return threshold;
      if (number < 0) return -threshold;
    }
    return number;
  }

  /**
   * Fonction threshold qui retourne 0 si le nombre est trop petit selon le threshold demandé
   *
   * @param threshold
   * @param number
   * @return le nombre avec le threshold appliqué
   */
  public static double threshold(double threshold, double number) {
    if (inRange(-threshold, threshold, number)) return 0;
    return number;
  }

  public static class DoubleCoerce {
    private double speedLeft;
    private double speedRight;
    private double maxSpeed;

    public DoubleCoerce() {}

    public void setSpeed(double speedLeft, double speedRight, double maxSpeed) {
      this.speedLeft = speedLeft;
      this.speedRight = speedRight;
      this.maxSpeed = maxSpeed;
      if (speedLeft < 0 && speedRight < 0) {
        maxSpeed *= -1;
      }
    }

    public double getSpeedLeft() {
      if (Math.abs(speedLeft) > maxSpeed) {
        if (Math.abs(speedRight) > Math.abs(speedLeft)) {
          return (speedLeft / speedRight) * maxSpeed;
        } else {
          return maxSpeed;
        }
      }
      return speedLeft;
    }

    public double getSpeedRight() {
      if (Math.abs(speedRight) > maxSpeed) {
        if (Math.abs(speedRight) > Math.abs(speedLeft)) {
          return (speedRight / speedLeft) * maxSpeed;
        } else {
          return maxSpeed;
        }
      }
      return speedRight;
    }
  }
}
