import subprocess
import argparse
from datetime import datetime, timedelta
import os


def run_simulation_for_date(script_name: str,
                            date_str: str,
                            python_executable: str,
                            increasing: bool = False,
                            schedule_csv: str = 'data/additional_depot_vehicles_schedule_template_v1.csv',
                            lunch_realloc: str | None = None,
                            lunch_duration: int = 60,
                            force_both: bool = False,
                            region_strict_ratio: float | None = None,
                            adjust_schedule: bool = False,
                            shift_rule: str | None = None,
                            ratio: float | None = None) -> bool:
    cmd = [python_executable, script_name, '--date', date_str]

    # scheduled 전용 옵션 전파
    if os.path.basename(script_name) == 'scheduled_increasing_with_shift_scenario_simulation.py':
        if increasing:
            cmd.append('--increasing')
            if schedule_csv:
                cmd.extend(['--schedule-csv', schedule_csv])
        if lunch_realloc:
            cmd.extend(['--lunch-realloc', lunch_realloc])
        if lunch_duration:
            cmd.extend(['--lunch-duration', str(lunch_duration)])
        if force_both:
            cmd.append('--force-both')
        if region_strict_ratio is not None:
            cmd.extend(['--region-strict-ratio', str(region_strict_ratio)])
        if adjust_schedule:
            cmd.append('--adjust-schedule')
        if shift_rule:
            cmd.extend(['--shift-rule', shift_rule])
        if ratio is not None:
            cmd.extend(['--ratio', str(ratio)])

    try:
        # 자식 프로세스의 상세 로그는 숨김
        result = subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except subprocess.CalledProcessError as e:
        return False
    except FileNotFoundError:
        print(f"Error: Python executable not found at '{python_executable}' or script '{script_name}' not found.")
        return False


def main():
    parser = argparse.ArgumentParser(description='Run daily simulations for a full month.')
    parser.add_argument('--year', type=int, required=True, help='Year (e.g., 2025)')
    parser.add_argument('--month', type=int, required=True, help='Month (1-12)')
    parser.add_argument('--script', type=str, choices=['baseline', 'scheduled'], default='scheduled',
                        help='Which simulation script to run')
    parser.add_argument('--python', type=str, default='python', help='Python executable path')

    # scheduled 전용 옵션
    parser.add_argument('--increasing', action='store_true', help='Enable additional scheduled vehicles (scheduled only)')
    parser.add_argument('--schedule-csv', type=str, default='data/additional_depot_vehicles_schedule_template_v1.csv',
                        help='Schedule CSV for additional vehicles (scheduled only)')
    parser.add_argument('--lunch-realloc', type=str, default=None,
                        help='Lunch reallocation rule, e.g., "12->11:30=0.8,12->13=0.2" (scheduled only)')
    parser.add_argument('--lunch-duration', type=int, default=60, help='Lunch duration in minutes (scheduled only)')
    parser.add_argument('--force-both', action='store_true', help='Force all vehicles as BOTH (scheduled only)')
    parser.add_argument('--region-strict-ratio', type=float, default=None, help='Same region assignment ratio 0..1 (scheduled only)')
    parser.add_argument('--adjust-schedule', action='store_true', help='Enable driver shift adjustment (scheduled only)')
    parser.add_argument('--shift-rule', type=str, default=None, help="Shift rule like '6to4' (scheduled only)")
    parser.add_argument('--ratio', type=float, default=None, help='Share of drivers to adjust 0.0~1.0 (scheduled only)')

    args = parser.parse_args()

    # 월의 시작/끝 계산
    start_date = datetime(args.year, args.month, 1)
    if args.month == 12:
        end_date = datetime(args.year + 1, 1, 1) - timedelta(days=1)
    else:
        end_date = datetime(args.year, args.month + 1, 1) - timedelta(days=1)

    # 스크립트 파일 매핑 (월간 스크립트 파일 위치 기준으로 해석)
    if args.script == 'baseline':
        script_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'baseline_scenario_simulation.py')
    else:
        script_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scheduled_increasing_with_shift_scenario_simulation.py')

    if not os.path.exists(script_file):
        print(f"Error: Simulation script '{script_file}' not found in the current directory.")
        return

    print(f"=== Running monthly simulations for {args.year}-{args.month:02d} with {script_file} ===")
    if args.script == 'scheduled':
        print(f"  increasing: {'ON' if args.increasing else 'OFF'} | schedule: {args.schedule_csv}")
        print(f"  lunch-realloc: {args.lunch_realloc if args.lunch_realloc else 'None'} | lunch-duration: {args.lunch_duration}m")
        print(f"  force-both: {'ON' if args.force_both else 'OFF'} | region-ratio: {args.region_strict_ratio if args.region_strict_ratio is not None else 'None'}")
        print(f"  shift: {'ON' if args.adjust_schedule else 'OFF'} | rule: {args.shift_rule if args.shift_rule else 'None'} | ratio: {args.ratio if args.ratio is not None else 'None'}")

    ok_days: list[str] = []
    bad_days: list[str] = []

    cur = start_date
    while cur <= end_date:
        date_str = cur.strftime('%Y-%m-%d')
        print(f"\n--- {date_str} ---")
        success = run_simulation_for_date(
            script_name=script_file,
            date_str=date_str,
            python_executable=args.python,
            increasing=args.increasing,
            schedule_csv=args.schedule_csv,
            lunch_realloc=args.lunch_realloc,
            lunch_duration=args.lunch_duration,
            force_both=args.force_both,
            region_strict_ratio=args.region_strict_ratio,
            adjust_schedule=args.adjust_schedule,
            shift_rule=args.shift_rule,
            ratio=args.ratio
        )
        (ok_days if success else bad_days).append(date_str)
        cur += timedelta(days=1)

    print('\n=== Monthly Summary ===')
    total_days = (end_date - start_date).days + 1
    print(f'Total: {total_days} days | Success: {len(ok_days)} | Failed: {len(bad_days)}')
    if bad_days:
        print('Failed dates:', ', '.join(bad_days))


if __name__ == '__main__':
    main()
