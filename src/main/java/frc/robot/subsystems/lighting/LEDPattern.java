package frc.robot.subsystems.lighting;

public abstract class LEDPattern implements Runnable {
  private String name;
  private boolean running;
  protected int tick = 0;
  protected int ticksPerSecond;
  private Thread t;

  public LEDPattern(int ticksPerSecond, String name) {
    this.name = name;
    running = true;
    this.ticksPerSecond = ticksPerSecond;
    if (!LEDStrip.isRunning()) {
      LEDStrip.active = this;
      run();
    } else {
      LEDStrip.active.stop();
      LEDStrip.active = this;
      run();
    }
  }

  public abstract void setPixels();

  public void stop() {
    running = false;
  }

  public boolean isRunning() {
    return running;
  }

  @Override
  public void run() {
    if (t == null) {
      t = new Thread(() -> {
        System.out.println("ahhhhh");
        long lastTime = System.nanoTime();
        double ns = 1000000000 / (double) ticksPerSecond;
        double delta = 0;
        running = true;

        while (running) {
          try {
            if (Thread.interrupted()) {
              throw new InterruptedException();
            }
          }
          catch (Exception e) {
            return;
          }
          long now = System.nanoTime();
          delta += (now - lastTime) / ns;
          lastTime = now;

          if (delta >= 1) {
            setPixels();
            LEDStrip.update();
            tick++;
            delta--;
          }
        }
      });
      t.start();
    }
  }
}

