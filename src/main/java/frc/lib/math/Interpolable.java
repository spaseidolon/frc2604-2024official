package frc.lib.math;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
