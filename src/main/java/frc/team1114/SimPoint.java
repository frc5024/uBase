package frc.team1114;

public class SimPoint {
	private double x;
	private double y;

	public SimPoint(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public SimPoint() {
		this(0, 0);
	}

	public void rotateByAngleDegrees(double angle) {
		angle = Math.toRadians(angle);

		double x_ = x * Math.cos(angle) - y * Math.sin(angle);
		double y_ = x * Math.sin(angle) + y * Math.cos(angle);

		this.x = x_;
		this.y = y_;
	}

	public double getX() {
		return this.x;
	}

	public double getY() {
		return this.y;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public String toString() {
		return "(" + x + ", " + y + ")";
	}
}
