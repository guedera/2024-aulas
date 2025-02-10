import CardNotificacao from "./card-notificacao";
import styles from './pop-up-header.module.css';

export default function PopUp(props) {
    return (
        <div className={styles.box}>
            {props.lista.length > 0 ? (
                props.lista.map((item, index) => (
                    <CardNotificacao key={index}>{item.mensagem}</CardNotificacao>
                ))
            ) : (
                <CardNotificacao key="empty">Não há nenhuma notificação</CardNotificacao>
            )}
        </div>
    );
}
