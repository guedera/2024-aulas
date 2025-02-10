import styles from './card-treino.module.css';

export default function CardTreino(props) {

    return (
        <div className={styles.cardTreino}>
            <div className={styles.containerDatas} >
                    <p>{props.data}</p>
                    <p>{props.hora}</p>
            </div>
            <p className={styles.series}>{props.numero} séries</p>
        </div>
    );
}