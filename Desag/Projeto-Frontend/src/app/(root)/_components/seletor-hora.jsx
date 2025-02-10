import styles from './seletor-hora.module.css';

export default function SeletorHora(props) {
    return (
        <div>
            <p>{props.name}</p>
            <input
                type="time"
                className={styles.seletorHora}
                name={props.name}
                value={props.value}
                onChange={props.onChange}
            />
        </div>
    );
}