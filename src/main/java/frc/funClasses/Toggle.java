import java.util.function.BooleanSupplier;

public class Toggle {
  boolean isInverted;
  boolean isPreviouslyPressed;
  boolean value;
  BooleanSupplier source;

  public Toggle(BooleanSupplier source) {
    this(source, false, false);
  }
  public Toggle(BooleanSupplier source, boolean inverted, boolean init) {
    this.source = source;
    this.isInverted = inverted;
    this.isPreviouslyPressed = false;
    this.value = init;
  }
  public boolean getToggle() {
    if (!isPreviouslyPressed && sourceGet()) {
      isPreviouslyPressed = true;
      value = !value;
    } else if (!sourceGet()) {
      isPreviouslyPressed = false;
    }
    return value;
  }
  public void setInverted(boolean inverted) {
    this.isInverted = inverted;
  }
  private boolean sourceGet() {
    return (isInverted) ? !source.getAsBoolean() : source.getAsBoolean();
  }
}
